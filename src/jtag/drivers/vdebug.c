//#----------------------------------------------------------------------------
//# Copyright 2020 Cadence Design Systems, Inc.
//#
//# Redistribution and use in source and binary forms, with or without modification,
//# are permitted provided that the following conditions are met:
//# 1. Redistributions of source code must retain the above copyright notice, 
//# this list of conditions and the following disclaimer.
//# 2. Redistributions in binary form must reproduce the above copyright notice,
//# this list of conditions and the following disclaimer in the documentation 
//# and/or other materials provided with the distribution.
//#----------------------------------------------------------------------------
//# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
//# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
//# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
//# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//#----------------------------------------------------------------------------
//# Revisions   :
//# 40 04.05.20 : based on vd_client and vd_test 
//#----------------------------------------------------------------------------

/*!
 * @file
 *  
 * @brief the virtual debug interface provides a connection between a sw debugger 
 * and the simulated, emulated core over a soft connection, implemented by DPI
 * It implements an interface and JTAG, DAP and AMBA transports
 * 
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#define SOCKET int
#ifdef HAVE_UNISTD_H
#include <unistd.h>          /* close */
#endif
#ifdef HAVE_SYS_SOCKET_H
#include <sys/socket.h>
#endif
#ifdef HAVE_ARPA_INET_H
#include <arpa/inet.h>
#endif
#ifdef HAVE_NETDB_H
#include <netdb.h>
#endif
#endif
#include <stdio.h>
#ifdef HAVE_STDINT_H
#include <stdint.h>
#endif
#ifdef HAVE_STDLIB_H
#include <stdlib.h>
#endif
#include <stdarg.h>
#include <string.h>
#include <errno.h>

#include "jtag/interface.h"
#include "jtag/commands.h"
#include "transport/transport.h"
#include "helper/log.h"

#define VD_VERSION 40
#define VD_BUFFER_LEN 4024
#define VD_CHEADER_LEN 24
#define VD_SHEADER_LEN 16

/**
 * @brief List of transactor types
 */
typedef enum {
  VD_BFM_JTDP  = 0x0001,  /**< transactor DAP JTAG DP */
  VD_BFM_SWDP  = 0x0002,  /**< transactor DAP SWD DP */
  VD_BFM_AHB   = 0x0003,  /**< transactor AMBA AHB */
  VD_BFM_APB   = 0x0004,  /**< transactor AMBA APB */
  VD_BFM_AXI   = 0x0005,  /**< transactor AMBA AXI */
  VD_BFM_JTAG  = 0x0006,  /**< transactor serial JTAG */
  VD_BFM_SWD   = 0x0007,  /**< transactor serial SWD */
} vd_bfm_et;

/**
 * @brief List of signals that can be read or written by the debugger
 */
typedef enum {
  VD_SIG_TCK   = 0x0001,  /**< JTAG clock; tclk */
  VD_SIG_TDI   = 0x0002,  /**< JTAG TDI;   tdi */
  VD_SIG_TMS   = 0x0004,  /**< JTAG TMS;   tms */
  VD_SIG_RESET = 0x0008,  /**< DUT reset;  rst */
  VD_SIG_TRST  = 0x0010,  /**< JTAG Reset; trstn */
  VD_SIG_TDO   = 0x0020,  /**< JTAG TDO;   tdo */
  VD_SIG_POWER = 0x0100,  /**< BFM power;  bfm_up */
  VD_SIG_TCKDIV= 0x0200,  /**< JTAG clock divider; tclkdiv */
  VD_SIG_BUF   = 0x1000,  /**< memory buffer; mem */
} vd_sig_et;

/**
 * @brief List of errors
 */
typedef enum {
  VD_ERR_NONE      = 0x0000,  /**< no error */
  VD_ERR_NOT_IMPL  = 0x0100,  /**< feature not implemented */
  VD_ERR_USAGE     = 0x0101,  /**< incorrect usage */
  VD_ERR_PARAM     = 0x0102,  /**< incorrect parameter */
  VD_ERR_CONFIG    = 0x0107,  /**< incorrect configuration */
  VD_ERR_NO_MEMORY = 0x0104,  /**< out of memory */
  VD_ERR_SHM_OPEN  = 0x010a,  /**< cannot open shared memory */
  VD_ERR_SHM_MAP   = 0x010b,  /**< cannot map shared memory */
  VD_ERR_SOC_OPEN  = 0x011a,  /**< cannot open socket */
  VD_ERR_SOC_OPT   = 0x011b,  /**< cannot set socket option */
  VD_ERR_SOC_ADDR  = 0x011c,  /**< cannot resolve host address */
  VD_ERR_SOC_CONN  = 0x011d,  /**< cannot connect to host */
  VD_ERR_SOC_SEND  = 0x011e,  /**< error sending data on socket */
  VD_ERR_SOC_RECV  = 0x011f,  /**< error receiving data from socket */
  VD_ERR_LOCKED    = 0x0202,  /**< device locked */
  VD_ERR_NOT_RUN   = 0x0204,  /**< transactor not running */
  VD_ERR_NOT_OPEN  = 0x0205,  /**< transactor not open/connected */
  VD_ERR_LICENSE   = 0x0206,  /**< cannot check out the license */
  VD_ERR_VERSION   = 0x0207,  /**< transactor version mismatch */
  VD_ERR_TIME_OUT  = 0x0301,  /**< time out, waiting */
  VD_ERR_NO_POWER  = 0x0302,  /**< power out error */
  VD_ERR_BUS_ERROR = 0x0304,  /**< bus protocol error, like pslverr */
  VD_ERR_NO_ACCESS = 0x0306,  /**< no access to an object */
  VD_ERR_INV_HANDLE= 0x0307,  /**< invalid object handle */
  VD_ERR_INV_SCOPE = 0x0308,  /**< invalid scope */
} vd_err_et;

typedef struct
{
  struct
  {                          // VD_CHEADER_LEN written by client
    uint8_t cmd;             // 000;
    uint8_t type;            // 001;
    uint16_t waddr;          // 002;
    uint16_t wbytes;         // 004;
    uint16_t rbytes;         // 006;
    uint16_t wwords;         // 008;
    uint16_t rwords;         // 00a;
    uint32_t rwdata;         // 00c;
    uint32_t offset;         // 010;
    uint16_t offseth;        // 014;
    uint16_t wid;            // 016;
  };
  union
  {                          // 018;
    uint8_t wd8[VD_BUFFER_LEN];
    uint32_t wd32[VD_BUFFER_LEN/4];
    uint64_t wd64[VD_BUFFER_LEN/8];
  };
  struct
  {                          // VD_SHEADER_LEN written by server
    uint16_t rid;            // fd0: 
    uint16_t awords;         // fd2:
    int32_t  status;         // fd4;
    uint64_t duttime;        // fd8;
  };
  union
  {                          // fe0:
    uint8_t rd8[VD_BUFFER_LEN];
    uint32_t rd32[VD_BUFFER_LEN/4];
    uint64_t rd64[VD_BUFFER_LEN/8];
  };
  uint32_t state;            // 1f98;
  uint32_t count;            // 1f9c;
  uint8_t dummy[96];         // 1fa0; 48+40B+8B;
} vd_shm_st;


static FILE* debug_log = NULL;
static uint16_t debug = 0;
static uint8_t debug_out = 0;
static uint8_t trans_first = 0;
static uint8_t trans_last = 0;

static uint32_t bfm_period;
static uint8_t buf_width;
static uint8_t addr_bits;
static uint8_t bfm_type;
static uint16_t sig_read;
static uint16_t sig_write;
static uint32_t mem_base;
//static uint32_t mem_width;
//static uint32_t mem_depth;
static uint32_t server_port;
static uint32_t pollcycles = 10000;
static uint32_t pollmin;
static uint32_t pollmax;
static uint64_t polltime;
static uint32_t pollreqs = 0;
static int hsocket;
static char server_name[32];
static char bfm_path[128];
static char mem_path[128];
static vd_shm_st* pbuf = NULL;
static const char* const vdebug_transports[] = { "jtag", NULL };

//static int (*targ_poll)(struct target *target) = NULL;
//static int (*targ_read_buffer)(struct target *target, uint32_t address, uint32_t size, uint8_t *buffer) = NULL;
//static int (*targ_write_buffer)(struct target *target, uint32_t address, uint32_t size, const uint8_t *buffer) = NULL;

static void debug_open(void)
{
  char msg[32];
  char *p = getenv( "VD_DEBUG" );
  debug = (p ? strtol( p, NULL, 0 ) : 0);
  if( debug )
  {
    if( (p = getenv( "VD_LOG" )) == NULL )
    {
      strcpy( msg, "vd_client.log" );
      debug_log = fopen( msg, "w" );
    }
    else
      debug_log = fopen( p, "w" );
    if( debug > 0x100 )
    {
      debug_out = debug >> 8;
      debug &= 0x00ff;
    }
    sprintf( msg, "OpenOCD vdebug client %d", VD_VERSION );
    if( debug_log )
    {
      fputs( msg, debug_log ); fputc( '\n', debug_log );
    }
  }
}

static uint32_t debug_msg( uint32_t er_code, char* format, ...)
{
  char msg[128];
  long long unsigned duttime = (pbuf ? pbuf->duttime : 0);
  if (format)
  {
    va_list ap;
    snprintf( msg, 15, "%10lluns: ", duttime );
    va_start(ap,format);     // explicitly terminating string, Windows bug
    vsnprintf(msg+14, sizeof(msg)-15, format, ap); msg[127]='\0';
    if( er_code )
      LOG_ERROR("%s: vdebug code 0x%x", msg, er_code);
    else if( debug_out )
      LOG_DEBUG("%s", msg);
    va_end(ap);
    if( debug && debug_log )
    {
      fputs( msg, debug_log ); fputc( '\n', debug_log );
    }
  }
  return er_code;
}

static int socket_error(void)
{
#ifdef _WIN32
  return WSAGetLastError();
#else
  return errno;
#endif
}

static int socket_close(SOCKET hsock)
{
#ifdef _WIN32
  closesocket(hsock);
  WSACleanup();
#else
  close(hsock);
#endif
  return 0;
}

static SOCKET socket_open(char* server_addr, uint32_t port)
{
  SOCKET hsock;
  int rc = 0;
  uint32_t buflen = sizeof(vd_shm_st); // size of the send and rcv buffer
  struct addrinfo *ainfo = NULL;
  struct addrinfo ahint = { 0, AF_INET, SOCK_STREAM, 0, 0, NULL, NULL, NULL };

#ifdef _WIN32
  WSADATA ver;
  if ((rc = WSAStartup(MAKEWORD(2, 2), &ver)) != 0)
    ;
  else if ((hsock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP)) == INVALID_SOCKET)
    rc = debug_msg( VD_ERR_SOC_OPEN, "socket_open: cannot open socket, error %d", socket_error() );
#else
  uint32_t rcvwat = VD_SHEADER_LEN;    // size of the rcv header, as rcv min watermark
  if ((hsock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP)) < 0)
    rc = debug_msg( VD_ERR_SOC_OPEN, "socket_open: cannot open socket, error %d", socket_error() );
  else if (setsockopt(hsock, SOL_SOCKET, SO_RCVLOWAT, &rcvwat, sizeof(rcvwat)) < 0)
    rc = errno;
#endif
  else if (setsockopt(hsock, SOL_SOCKET, SO_SNDBUF, (const char*)&buflen, sizeof(buflen)) < 0)
    rc = socket_error();
  else if (setsockopt(hsock, SOL_SOCKET, SO_RCVBUF, (const char*)&buflen, sizeof(buflen)) < 0)
    rc = socket_error();
  if (rc)
    rc = debug_msg( VD_ERR_SOC_OPT, "socket_open: cannot set socket option, error %d", rc );
  else if ((rc = getaddrinfo(server_addr, NULL, &ahint, &ainfo)) != 0)
    rc = debug_msg( VD_ERR_SOC_ADDR, "socket_open: cannot resolve address %s, error %d", server_addr, rc );
  else
  {
    ((struct sockaddr_in*)(ainfo->ai_addr))->sin_port = htons(port);
    if (connect(hsock, ainfo->ai_addr, sizeof(struct sockaddr)) < 0)
      rc = debug_msg( VD_ERR_SOC_CONN, "socket_open: cannot connect to %s:%d, error %d", server_addr, port, socket_error() );
  }
  if (rc)
  {
    socket_close(hsock);
    hsock = 0;
  }
  if (ainfo)
    freeaddrinfo(ainfo);
  return hsock;
}

static int socket_receive(SOCKET hsock, vd_shm_st* pmem)
{
  int rc;
  uint16_t dreceived = 0;
  uint16_t offset = (uint8_t*)&pmem->rid - &pmem->cmd;
  uint16_t to_receive = VD_SHEADER_LEN + pmem->rbytes;
  char *pb = (char*)pmem;

  do
  {
    if ((rc = recv(hsock, pb + offset, to_receive, 0)) <= 0)
      break;
    else
    {              // the data can come in pieces
      to_receive -= rc;
      offset += rc;
    }
    if (debug > 2)
      debug_msg( VD_ERR_NONE, "socket_receive: received %u, to receive %u", rc, to_receive );
    dreceived += rc;
  } while (rc > 0 && to_receive);
  if (rc <= 0)
    debug_msg( VD_ERR_SOC_RECV, "socket_receive: recv failed, error %d", socket_error() );
  else
    rc = dreceived;
  return rc;
}

static int socket_send(SOCKET hsock, vd_shm_st* pmem)
{
  int rc;
  if( (rc = send(hsock, (const char*)&pmem->cmd, VD_CHEADER_LEN + pmem->wbytes, 0)) <= 0)
    debug_msg( VD_ERR_SOC_SEND, "socket_send: send failed, error %d", socket_error() );
  else if (debug > 2)
    debug_msg( VD_ERR_NONE, "socket_send: sent %u, to send %u", rc, 0 );
  return rc;
}

static uint32_t wait_server( SOCKET hsock, vd_shm_st* pmem )
{
  int st, rd;
  if( !hsock )
    st = VD_ERR_SOC_OPEN;
  else if( (st = socket_send(hsock, pmem)) <= 0 )
    st = VD_ERR_SOC_SEND;
  else if( (rd = socket_receive(hsock, pmem)) <= 0 )
    st = VD_ERR_SOC_RECV;
  else
  {
    if( debug > 3 )
      debug_msg( 0, "wait_server: cmd %02hx done, sent %d, rcvd %d, status %d time %llu", pmem->cmd, st, rd, pmem->status, pmem->duttime );
    st = pmem->status;
  }
  return st;
}

static int vdebug_open( SOCKET hsock, vd_shm_st* pm, const char* path, uint32_t type, uint32_t period_ps, uint32_t sig_mask )
{
  int rc;
  
  if( pm )
  {
    pm->cmd = 0x01;
    pm->wid = (uint16_t)VD_VERSION;
    pm->wbytes = pm->rbytes = pm->wwords = pm->rwords = 0;
    if( (rc = wait_server( hsock, pm )) != 0 ) // communication problem
      debug_msg( rc, "vd_open: Error %x connecting to server", rc );
    else if( pm->rid < pm->wid )       // communication OK, but version wrong
    {
      debug_msg( VD_ERR_VERSION, "vd_open: server version %d too old for the client %d", pm->rid, pm->wid );
      pm->cmd = 0x02;                  // let server close the connection
      wait_server( hsock, pm );
    }
    else
    {
      pm->cmd = 0x04;
      pm->type = (uint8_t)type;
      pm->rwdata = sig_mask | VD_SIG_BUF | (VD_SIG_BUF << 16);
      pm->wbytes = (uint16_t)strlen(path)+1; pm->rbytes = 12;
      pm->wid = 0;             // reset wid for transaction ID
      pm->wwords = pm->rwords = 0;
      memcpy( pm->wd8, path, pm->wbytes+1 );
      rc = wait_server( hsock, pm );
      bfm_type = type;
      sig_read = (uint16_t)(pm->rwdata >> 16);    // signal read mask
      sig_write = (uint16_t)pm->rwdata;   // signal write mask
      bfm_period = period_ps;
      buf_width = pm->rd32[0]/8;// access width in bytes
      addr_bits = pm->rd32[2];
    }
    if( rc )
      debug_msg( rc, "vd_open: Error %x connecting to BFM %s", rc, path ); 
    else if( debug )
      debug_msg( VD_ERR_NONE, "vd_open: %s type %0x, period %dps, buffer %dx%dB signals r%04xw%04x",
      path, bfm_type, bfm_period, VD_BUFFER_LEN/buf_width, buf_width, sig_read, sig_write );
  }
  else
    rc = VD_ERR_NOT_OPEN;
  return rc;
}

static int vdebug_close( SOCKET hsock, vd_shm_st* pm, uint32_t type )
{
  int rc = VD_ERR_NOT_OPEN;
  if( pm )
  {
    pm->cmd = 0x05;
    pm->type = (uint8_t)type;
    pm->wbytes = pm->rbytes = pm->wwords = pm->rwords = 0;
    rc = wait_server( hsock, pm );
    pm->cmd = 0x02;
    pm->wid = (uint16_t)VD_VERSION;
    pm->wbytes = pm->rbytes = pm->wwords = pm->rwords = 0;
    wait_server( hsock, pm );
    if( debug )
      debug_msg( VD_ERR_NONE, "vd_close: type %0x", type );
  }
  return rc;
}

static int vdebug_wait( SOCKET hsock, vd_shm_st* pm, uint32_t cycles )
{
  int rc = VD_ERR_NOT_OPEN;
  if( pm && cycles )
  {
    pm->cmd = 0x08;
    pm->wbytes = 4; pm->wwords = 1;
    pm->rbytes = 0; pm->rwords = 0;
    pm->wd32[0] = cycles;
    rc = wait_server( hsock, pm );
    if( debug )
      debug_msg( VD_ERR_NONE, "vd_wait: %d cycles", cycles );
  }
  return rc;
}

static int vdebug_sig_set( SOCKET hsock, vd_shm_st* pm, uint32_t write_mask, uint32_t value )
{
  int rc = VD_ERR_NOT_OPEN;
  if( pm )
  {
    pm->cmd = 0x11;
    pm->wbytes = 4; pm->wwords = 1;
    pm->rbytes = 0; pm->rwords = 0;
    pm->rwdata = write_mask;
    pm->wd32[0] = value;
    rc = wait_server( hsock, pm );
    if( rc ) 
      debug_msg( rc, "vd_sig_set: Error %x setting signals %04x", rc, write_mask );
    else if( debug )
      debug_msg( VD_ERR_NONE, "vd_sig_set: setting signals %04x to %04x", write_mask, value );
  }
  return rc;
}

static int vdebug_jtag_clock( SOCKET hsock, vd_shm_st* pm, uint32_t value )
{
  int rc = VD_ERR_NOT_OPEN;
  if( pm )
  {
    pm->cmd = 0x19;
    pm->wbytes = 4; pm->wwords = 1;
    pm->rbytes = 0; pm->rwords = 0;
    pm->wd32[0] = value;
    rc = wait_server( hsock, pm );
    if( rc ) 
      debug_msg( rc, "vd_jtag_clock: Error %x setting jtag_clock", rc );
    else if( debug )
      debug_msg( VD_ERR_NONE, "vd_jtag_clock: setting jtag clock divider to %d", value );
  }
  return rc;
}

static int vdebug_jtag_shift_tap( SOCKET hsock, vd_shm_st* pm, uint32_t num_pre, const uint8_t tms_pre, uint32_t num, const uint8_t* tdi, uint32_t num_post, const uint8_t tms_post, uint8_t* tdo, uint8_t f_last )
{
  const uint32_t tobits = 8;
  uint16_t i,j;
  uint16_t bytes, hwords, anum, words, waddr;
  uint32_t tdi_last;
  int rc = 0;
  
  if( pm )
  {
    pm->cmd = 0x18;
    trans_last = f_last;
    if( trans_first )
      waddr = 0;             // reset buffer offset
    else
      waddr = pm->waddr;     // continue from the previous transaction
    anum = num + num_pre + num_post;   // actual number of bits to shift
    hwords = (anum+4*buf_width-1)/(4*buf_width); // in 4B TDI/TMS words
    words = (hwords+1)/2;    // in 8B TDO words to read
    bytes = (num+7)/8;       // data only portion in bytes
                             // buffer overflow check and flush
    if( waddr + 2 + hwords > VD_BUFFER_LEN/4 )
    {                        // buffer overflow
      anum = 0;              // force flush
      trans_last = 1;        // end return an error
      debug_msg( VD_ERR_NONE, "vdebug_jtag_shift_tap: %04x Error, buffer overflow L:%02d O:%05x @%04x", pm->wid, anum, ((trans_first << 14)|(trans_last << 15)), waddr );
    }
    else if( waddr + 2 + hwords + 4 > VD_BUFFER_LEN/4 )
      trans_last = 1;        // no room for next 4W transaction in buffer, force flush

    if( anum )               // support for calls with num=0 as flush
    {
      pm->wd32[waddr++] = anum + (tdo ? 0xc0000000: 0x40000000);
      pm->wd32[waddr++] = hwords + (tdo ? (words << 16): 0);
      pm->wid++;
      pm->wd8[4*waddr] = (tdi ? (tdi[0] << num_pre) : 0);
      pm->wd8[4*waddr+4] = tms_pre;    // init with tms_pre
      if( num+num_pre <= 8 )           // and tms_post for num <=4
        pm->wd8[4*waddr+4] |= (tms_post << (num+num_pre-1));
      for( i=1,j=4*waddr; i<bytes; i++ )  // copy the tdi and tms data
      {
        if( i == bytes-1 && num+num_pre-1 <= bytes*tobits )
          pm->wd8[j+i+4] = tms_post << ((num+num_pre-1) % 8);
        else
          pm->wd8[j+i+4] = 0x0;// placing 4 bytes of TMS bits into high word
        if( !tdi )             // placing 4 bytes of TDI bits into low word
          pm->wd8[j+i] = 0x0;
        else
          pm->wd8[j+i] = (tdi[i] << num_pre) | (tdi[i-1] >> (8-num_pre));
        if( i % 4 == 3 )
        {
          if( debug > 2 )
            debug_msg( VD_ERR_NONE, "vdebug_jtag_shift_tap: %04x DI[%02x]:%08x MS[%02x]:%08x", pm->wid-1, (i+j)/4, pm->wd32[(i+j)/4], (i+j+4)/4, pm->wd32[(i+j+4)/4] );
          j += 4;
        }
      }
      if( tdi )
      {
        if( num+num_pre > bytes*tobits )// in case 1 additional byte needed for TDI
          pm->wd8[j+i] = (tdi[i-1] >> (8-num_pre)); // put last TDI bits there
        tdi_last = *(uint32_t*)(tdi + ((bytes-1)/4)*4);
      }
      else
        tdi_last = 0;
      if( num+num_pre <= bytes*tobits )// in case no or 1 additional byte needed 
        pm->wd8[j+i+4] = tms_post >> (8-(num+num_pre-1) % 8); // may need to add higher part
                                       // in case exactly 1 additional byte needed
      else if( num+num_pre > bytes*tobits && num+num_pre+num_post <= (bytes+1)*tobits )
        pm->wd8[j+i+4] = tms_post << ((num+num_pre-1) % 8); // add whole tms_post
      else                             // in case 2 additional bytes, tms_post split
      {
        pm->wd8[j+i+4] = tms_post << ((num+num_pre-1) % 8);// add lower part of tms_post
        if( i % 4 == 3 )               // next byte is in the next 32b word
          pm->wd8[j+i+4+5] = tms_post >> (8-(num+num_pre-1) % 8); // and higher part
        else                           // next byte is in the same 32b word
          pm->wd8[j+i+4+1] = tms_post >> (8-(num+num_pre-1) % 8); // and higher part
      }
      if( debug > 2 )
        debug_msg( VD_ERR_NONE, "vdebug_jtag_shift_tap: %04x DI[%02x]:%08x MS[%02x]:%08x", pm->wid-1, (i+j)/4, pm->wd32[(i+j)/4], (i+j+4)/4, pm->wd32[(i+j+4)/4] );
    }
    else if( debug )                   // flush, show options and waddr 
      debug_msg( VD_ERR_NONE, "vdebug_jtag_shift_tap: %04x L:%02d O:%05x @%04x flush", pm->wid-1, num, ((trans_first << 14)|(trans_last << 15)), waddr );

    if( trans_last && waddr )
    {
      pm->rwords = (tdo ? words : 0);
      pm->wwords = waddr/2 + hwords;   // payload size *2 to include both TDI and TMS data
      pm->wbytes = pm->wwords*8;
      pm->rbytes = (tdo ? (anum+7)/8 : 0);
      pm->offseth = 0;
      if( debug > 3 )
        debug_msg( VD_ERR_NONE, "vdebug_jtag_shift_tap: %04x l:%02d hw:%02x rw:%02x bt:%02x ww:%02x wb:%02x @%04x", pm->wid-1, num, hwords, words, bytes, pm->wwords, pm->wbytes, waddr );
      rc = wait_server( hsock, pm );
      pm->waddr = 0;         // reset buffer write address
    }
    else if( waddr )
      pm->waddr = waddr + hwords*2;    // offset for next transaction, must be even
      
    if( rc )
      debug_msg( rc, "vdebug_jtag_shift_tap: Error %x executing transaction", rc );
    else if( tdo )
    {
      for( i=0; i<bytes; i++ )
      {
        tdo[i] = (pm->rd8[i] >> num_pre) | (pm->rd8[i+1] << (8-num_pre));
        if( (debug > 2) && (i % 4 == 3) )
          debug_msg( VD_ERR_NONE, "vdebug_jtag_shift_tap: %04x D0[%02x]:%08x", pm->wid-1, i/4, *(uint32_t*)(tdo + (i/4)*4) );
      }
      if( debug )
        debug_msg( VD_ERR_NONE, "vdebug_jtag_shift_tap: %04x L:%02d O:%05x @%04x DI:%08x DO:%08x", pm->wid-1, num, ((trans_first << 14)|(trans_last << 15)), waddr, tdi_last, *(uint32_t*)(tdo + ((bytes-1)/4)*4) );
    }
    else if( debug )
      debug_msg( VD_ERR_NONE, "vdebug_jtag_shift_tap: %04x L:%02d O:%05x @%04x DI:%08x MS:%08x", pm->wid-1, num, ((trans_first << 14)|(trans_last << 15)), waddr, tdi_last, (tms_post << (num_pre+num-1)) + tms_pre );

    trans_first = trans_last;          // flush forces trans_first flag
  }
  else
    rc = VD_ERR_NOT_OPEN;
  return rc;
}

static int vdebug_init(void)
{
  uint32_t type, sig_mask;
  char* pchar;
  int rc = ERROR_OK;

  debug_open();
  if( (pchar = getenv( "VD_POLL" )) == NULL )
    rc = 1;                  // default 1 per second
  else if( ((rc = strtoul( pchar, NULL, 10 )) > 5000) || (rc == 0) )
    rc = 1;                  // invalid, possibly old value, revert to default
  pollmin = 1000/(2*rc); pollmax = 1000/(rc); pollcycles = 10000/rc; polltime = pollmax;
  pollreqs = 0;
  type = VD_BFM_JTAG;
  sig_mask = VD_SIG_RESET | VD_SIG_TRST | VD_SIG_TCKDIV;

  if( (hsocket = socket_open( server_name, server_port )) <= 0 )
    rc = VD_ERR_SOC_OPEN;
  else if( (pbuf = (vd_shm_st*)calloc( 1, sizeof( vd_shm_st ))) == NULL )
  {
    socket_close( hsocket );
    hsocket = 0;
    LOG_ERROR("cannot allocate %lu bytes", sizeof( vd_shm_st ) );
  }
  else if( (rc = vdebug_open( hsocket, pbuf, bfm_path, type, bfm_period, sig_mask)) != 0 )
    ;
  else
    LOG_INFO("vdebug connected to %s through %s:%d", bfm_path, server_name, server_port);
  trans_first = 1;
  return rc;
}

static int vdebug_quit(void)
{
  int rc;
  rc = vdebug_close( hsocket, pbuf, VD_BFM_JTAG );
  if( hsocket )
    socket_close( hsocket );
  if( pbuf )
    free( pbuf );
  if( debug_log )
    fclose( debug_log );
  pbuf = NULL;
  hsocket = -1;
  debug_log = NULL;
  LOG_INFO("vdebug disconnected from %s through %s:%d", bfm_path, server_name, server_port);
  return rc;
}

static int vdebug_reset(int trst, int srst)
{
  uint16_t sig_val = 0xffff;
  uint16_t sig_mask = 0;
  int rc;

  sig_mask |= VD_SIG_RESET;
  if( srst )
    sig_val &= ~VD_SIG_RESET;// active low
  if( transport_is_jtag() )
  {
    sig_mask |= VD_SIG_TRST;
    if( trst )
      sig_val &= ~VD_SIG_TRST; // active low  
  }
  LOG_INFO("rst trst:%d srst:%d mask:%x val:%x", trst, srst, sig_mask, sig_val);
  if( (rc = vdebug_sig_set( hsocket, pbuf, sig_mask, sig_val )) != 0 )
    ;
  else
    rc = vdebug_wait( hsocket, pbuf, 20 );

  return rc;
}

static int vdebug_tms_seq(const uint8_t *bits, int nb_bits)
{
  LOG_INFO("TMS  len:%d tms:%x", nb_bits, *(const uint32_t*)bits);
  return vdebug_jtag_shift_tap( hsocket, pbuf, nb_bits, *bits, 0, NULL, 0, 0, NULL, 0 );
}

static int vdebug_path_move(struct pathmove_command *cmd)
{
  uint8_t trans[DIV_ROUND_UP(cmd->num_states, 8)];
  LOG_INFO("path num states %d", cmd->num_states );

  memset(trans, 0, DIV_ROUND_UP(cmd->num_states, 8));

  for (int i = 0; i < cmd->num_states; i++) {
    if (tap_state_transition(tap_get_state(), true) == cmd->path[i])
      buf_set_u32(trans, i, 1, 1);
    tap_set_state(cmd->path[i]);
  }
  return vdebug_tms_seq(trans, cmd->num_states);
}

static int vdebug_tms(struct tms_command *cmd)
{
  return vdebug_tms_seq(cmd->bits, cmd->num_bits);
}

static int vdebug_tlr(tap_state_t state)
{
  int rc = ERROR_OK;
  uint8_t tms_pre;
  uint8_t num_pre;
  uint8_t cur;
  
  cur = tap_get_state();
  tms_pre = tap_get_tms_path( cur, state );
  num_pre = tap_get_tms_path_len( cur, state );
  LOG_INFO("tlr  from %x to %x", cur, state );
  if (cur != state)
  {
    rc = vdebug_jtag_shift_tap( hsocket, pbuf, num_pre, tms_pre, 0, NULL, 0, 0, NULL, 0 );
    tap_set_state(state);
  }
  return rc;
}

static int vdebug_scan(struct scan_command *cmd, uint8_t f_flush)
{
  int nb_bits;
  int rc;
  uint8_t tms_pre, tms_post; // tms value pre and post shift
  uint8_t num_pre, num_post; // num bits pre shift, post shift
  uint8_t state;
  uint8_t cur;
  uint8_t *pb;
  uint8_t *po;

  cur = tap_get_state();
  state = cmd->ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT;
  tms_pre = tap_get_tms_path( cur, state );
  num_pre = tap_get_tms_path_len( cur, state );
  tms_post = tap_get_tms_path( state, cmd->end_state );
  num_post = tap_get_tms_path_len( state, cmd->end_state );
  nb_bits = jtag_build_buffer(cmd, &pb);
  if( jtag_scan_type( cmd ) & SCAN_IN )
  {                          // read TDO when needed 
    po = pb;
    f_flush |= 1;            // force flush on read now...
  }
  else
    po = NULL;
  LOG_DEBUG("scan len:%d ir/!dr:%d state cur:%x end:%x", nb_bits, cmd->ir_scan, cur, cmd->end_state );
  rc = vdebug_jtag_shift_tap( hsocket, pbuf, num_pre, tms_pre, nb_bits, pb, num_post, tms_post, po, f_flush );
  if( !rc )
  {
    jtag_read_buffer(pb, cmd);
    if( cur != cmd->end_state )
      tap_set_state(cmd->end_state);
  }
  if( pb )
    free( pb );
  return rc;
}

static int vdebug_runtest(int cycles, tap_state_t state)
{
  int rc;
  uint8_t tms_pre;
  uint8_t num_pre;
  uint8_t cur;
  
  cur = tap_get_state();
  tms_pre = tap_get_tms_path( cur, state );
  num_pre = tap_get_tms_path_len( cur, state );
  LOG_DEBUG("idle len:%d state cur:%x end:%x", cycles, cur, state );
  rc = vdebug_jtag_shift_tap( hsocket, pbuf, num_pre, tms_pre, cycles, NULL, 0, 0, NULL, 0 );
  if( cur != state )
    tap_set_state( state );
  return rc;
}

static int vdebug_stableclocks(int cycles)
{
  uint8_t tms_bits[4];
  int cycles_remain = cycles;
  int nb_bits;
  int retval;
  const int CYCLES_ONE_BATCH = sizeof(tms_bits) * 8;

  LOG_INFO("stab len:%d state cur:%x", cycles, tap_get_state() );
  assert(cycles >= 0);

  /* use TMS=1 in TAP RESET state, TMS=0 in all other stable states */
  memset(&tms_bits, (tap_get_state() == TAP_RESET) ? 0xff : 0x00, sizeof(tms_bits));

  /* send the TMS bits */
  while (cycles_remain > 0) {
    nb_bits = (cycles_remain < CYCLES_ONE_BATCH) ? cycles_remain : CYCLES_ONE_BATCH;
    retval = vdebug_tms_seq(tms_bits, nb_bits);
    if (retval != ERROR_OK)
      return retval;
    cycles_remain -= nb_bits;
  }

  return ERROR_OK;
}

static int vdebug_sleep(int us)
{
  int rc;
  
  LOG_INFO("sleep %d us", us );
  rc = vdebug_wait( hsocket, pbuf, us/1000 );
  return rc;
}

static int vdebug_speed(int speed)
{
  uint32_t divval, clkmax;
  int rc;
  
  clkmax = 1000000000/(bfm_period * 2); // kHz
  divval = clkmax/speed;
  LOG_INFO("jclk speed:%d kHz set, BFM divider %u", speed, divval );
  rc = vdebug_jtag_clock( hsocket, pbuf, divval );
  return rc;
}

static int vdebug_khz(int khz, int* jtag_speed)
{
  uint32_t divval, clkmax;
  
  clkmax = 1000000000/(bfm_period * 2); // kHz
  divval = khz ? clkmax/khz : 1;
  *jtag_speed = clkmax/divval;
  LOG_DEBUG("khz  speed:%d from khz:%d", *jtag_speed, khz );
  return ERROR_OK;
}

static int vdebug_div(int speed, int* khz)
{
  *khz = speed;
  LOG_DEBUG("div  khz:%d from speed:%d", *khz, speed );
  return ERROR_OK;
}

static int vdebug_execute_queue(void)
{
  struct jtag_command *cmd;
  int retval = ERROR_OK;

  for (cmd = jtag_command_queue; retval == ERROR_OK && cmd != NULL; cmd = cmd->next)
  {
    switch (cmd->type)
    {
      case JTAG_RESET:
        retval = vdebug_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
        break;
      case JTAG_RUNTEST:
        retval = vdebug_runtest(cmd->cmd.runtest->num_cycles, cmd->cmd.runtest->end_state);
        break;
      case JTAG_STABLECLOCKS:
        retval = vdebug_stableclocks(cmd->cmd.stableclocks->num_cycles);
        break;
      case JTAG_TLR_RESET:
        retval = vdebug_tlr(cmd->cmd.statemove->end_state);
        break;
      case JTAG_PATHMOVE:
        retval = vdebug_path_move(cmd->cmd.pathmove);
        break;
      case JTAG_TMS:
        retval = vdebug_tms(cmd->cmd.tms);
        break;
      case JTAG_SLEEP:
        retval = vdebug_sleep(cmd->cmd.sleep->us);
        break;
      case JTAG_SCAN:
        retval = vdebug_scan(cmd->cmd.scan, cmd->next == NULL);
        break;
    }
  }
  return retval;
}

COMMAND_HANDLER(vdebug_set_server)
{
  char* pchar;
  int rc = ERROR_FAIL;
  if (CMD_ARGC == 0)
  {
    strcpy( server_name, "localhost" );
    server_port = 8192;    
  }
  else if( (pchar = strchr( CMD_ARGV[0], ':' )) != NULL )
  {                          // server:port
    *pchar = '\0';
    strncpy( server_name, CMD_ARGV[0], sizeof(server_name)-1 );
    server_port = atoi( ++pchar );
    rc = ERROR_OK;
  }
  else
  {
    strncpy( server_name, CMD_ARGV[0], sizeof(server_name)-1 );
    server_port = 8192;
    rc = ERROR_OK;
  }
  LOG_DEBUG("vdebug server: %s port %u", server_name, server_port);
  return rc;
}

COMMAND_HANDLER(vdebug_set_mem)
{
  int rc = ERROR_FAIL;
  if (CMD_ARGC != 2)
    LOG_ERROR("mem_path <path> <base_address>");
  else
  {
    strncpy( mem_path, CMD_ARGV[0], sizeof(mem_path)-1 );
    COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], mem_base);
    rc = ERROR_OK;
    LOG_DEBUG("vdebug mem_path: %s @ 0x%08x", mem_path, mem_base);
  }
  return rc;
}

COMMAND_HANDLER(vdebug_set_bfm)
{
  int rc = ERROR_FAIL;
  char prefix;
  if (CMD_ARGC != 2)
    LOG_ERROR("vdebug bfm_path <path> <clk_period[p|n|u]s>");
  else
  {
    strncpy( bfm_path, CMD_ARGV[0], sizeof(bfm_path)-1 );
    if( sscanf( CMD_ARGV[1], "%u%cs*", &bfm_period, &prefix ) == 2 )
    {
      switch( prefix )
      {   
        case 'u': bfm_period *= 1000000;
          break;
        case 'n': bfm_period *= 1000;
          break;
        case 'p':
        default:
          break;
      }
      rc = ERROR_OK;
      LOG_DEBUG("vdebug bfm_path: %s clk_period %dps", bfm_path, bfm_period);
    }
  }
  return rc;
}

static const struct command_registration vdebug_command_handlers[] = 
{
  {
    .name = "server",
    .handler = &vdebug_set_server,
    .mode = COMMAND_CONFIG,
    .help = "set the vdebug server name or address",
    .usage = "server <host:port>",
  },
  {
    .name = "bfm_path",
    .handler = &vdebug_set_bfm,
    .mode = COMMAND_CONFIG,
    .help = "set the vdebug BFM hierarchical path",
    .usage = "bfm_path <path> <clk_period[p|n|u]s>",
  },
  {
    .name = "mem_path",
    .handler = &vdebug_set_mem,
    .mode = COMMAND_CONFIG,
    .help = "set the design memory for the code load",
    .usage = "mem_path <path> <base_address>",
  },
  COMMAND_REGISTRATION_DONE
};

struct jtag_interface vdebug_interface = 
{
  .name = "vdebug",
  .supported = DEBUG_CAP_TMS_SEQ,
  .transports = vdebug_transports,
  .execute_queue = vdebug_execute_queue,
  .speed = vdebug_speed,
  .khz = vdebug_khz,
  .speed_div = vdebug_div,
  .commands = vdebug_command_handlers,
  .init = vdebug_init,
  .quit = vdebug_quit,
};
