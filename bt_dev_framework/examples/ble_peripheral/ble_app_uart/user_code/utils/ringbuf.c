#include "ringbuf.h"
#include <string.h>

#define __min(x, y) (((x) < (y)) ? (x) : (y))


int ringbuf_init(struct ringbuf *p_rbuf, uint8_t *p_buf, uint32_t size)
{
    if (size == 0 || p_buf == NULL) {
        return RINGBUF_INVALID_PARAM;
    }
    
    p_rbuf->in   = 0;
    p_rbuf->out  = 0;
    p_rbuf->buf  = p_buf;
    p_rbuf->size = size;
    
    return RINGBUF_OK;
}

int ringbuf_putchar(struct ringbuf *p_rbuf, const char data)
{
    int in = p_rbuf->in;

    if (in != p_rbuf->out - 1) {
        if (in == p_rbuf->size - 1) {
            if (0 != p_rbuf->out) {
                p_rbuf->buf[in] = data;
                p_rbuf->in      = 0;
                return 1;
            }
        } else {
            p_rbuf->buf[in] = data;
            p_rbuf->in++;
            return 1;
        }
    }

    return RINGBUF_OK;
}

int ringbuf_getchar(struct ringbuf *p_rbuf, uint8_t *p_data)
{
    int out = p_rbuf->out;

    if (out != p_rbuf->in) {
        *p_data = p_rbuf->buf[out];
        p_rbuf->out = ++out >= p_rbuf->size ? 0 : out;
        return 1;
    }

    return RINGBUF_OK;
}


uint32_t ringbuf_put(struct ringbuf *p_rbuf, const uint8_t *p_buf, uint32_t nbytes)
{
    int out = p_rbuf->out;
    int in;
    int bytes_put;
    int bytes_tmp;

    if (out > p_rbuf->in) {
        /* out is ahead of in.  We can fill up to two bytes before out */

        bytes_put = __min(nbytes, out - p_rbuf->in - 1);
        memcpy(&p_rbuf->buf[p_rbuf->in], p_buf, bytes_put);
        p_rbuf->in += bytes_put;

    } else if (out == 0) {
        /*
         * out is at the beginning of the buffer.  We can fill till
         * the next-to-last element
         */

        bytes_put = __min(nbytes, p_rbuf->size - p_rbuf->in - 1);
        memcpy(&p_rbuf->buf[p_rbuf->in], p_buf, bytes_put);
        p_rbuf->in += bytes_put;

    } else {
        /*
         * out has wrapped around, and its not 0, so we can fill
         * at least to the size of the ring buffer.  Do so, then see if
         * we need to wrap and put more at the beginning of the buffer.
         */

        bytes_put = __min(nbytes, p_rbuf->size - p_rbuf->in);
        memcpy(&p_rbuf->buf[p_rbuf->in], p_buf, bytes_put);
        in = p_rbuf->in + bytes_put;

        if (in == p_rbuf->size) {
            /* We need to wrap, and perhaps put some more chars */

            bytes_tmp = __min(nbytes - bytes_put, out - 1);
            memcpy(p_rbuf->buf, p_buf + bytes_put, bytes_tmp);
            p_rbuf->in = bytes_tmp;
            bytes_put += bytes_tmp;
        } else {
            p_rbuf->in = in;
        }
    }

    return (bytes_put);
}

uint32_t ringbuf_get(struct ringbuf *p_rbuf, uint8_t *p_buf, uint32_t nbytes)
{
    int in = p_rbuf->in;
    int out;
    int bytes_got;
    int bytes_tmp;

    if (in >= p_rbuf->out) {
        /* in has not wrapped around */

        bytes_got = __min(nbytes, in - p_rbuf->out);
        memcpy(p_buf, &p_rbuf->buf[p_rbuf->out], bytes_got);
        p_rbuf->out += bytes_got;

    } else {
        /*
         * in has wrapped around.  Grab chars up to the size of the
         * buffer, then wrap around if we need to.
         */

        bytes_got = __min(nbytes, p_rbuf->size - p_rbuf->out);
        memcpy(p_buf, &p_rbuf->buf[p_rbuf->out], bytes_got);
        out = p_rbuf->out + bytes_got;

        /*
         * If out is equal to size, we've read the entire buffer,
         * and need to wrap now.  If bytes_got < nbytes, copy some more chars
         * in now.
         */

        if (out == p_rbuf->size) {
            bytes_tmp = __min(nbytes - bytes_got, in);
            memcpy(p_buf + bytes_got, p_rbuf->buf, bytes_tmp);
            p_rbuf->out = bytes_tmp;
            bytes_got += bytes_tmp;
        } else {
            p_rbuf->out = out;
        }
    }

    return (bytes_got);
}

uint32_t ringbuf_put_with_length(struct ringbuf *p_rbuf, const uint8_t *p_buf, uint32_t nbytes)
{
    uint32_t bytes_put = 0;
    
    bytes_put += ringbuf_put(p_rbuf, (const uint8_t *)&nbytes, sizeof(uint32_t));
    bytes_put += ringbuf_put(p_rbuf, p_buf, nbytes);
    
    return bytes_put;
}

uint32_t ringbuf_get_with_length(struct ringbuf *p_rbuf, uint8_t *p_buf, uint32_t *p_nbytes)
{
    uint32_t nbytes, bytes_got = 0;
    
    bytes_got += ringbuf_get(p_rbuf, (uint8_t *)&nbytes, sizeof(uint32_t));
    bytes_got += ringbuf_get(p_rbuf, p_buf, nbytes);
    
    *p_nbytes = nbytes;
    
    return bytes_got;
}



void ringbuf_flush(struct ringbuf *p_rbuf)
{
    p_rbuf->in  = 0;
    p_rbuf->out = 0;
}


bool ringbuf_isempty(struct ringbuf *p_rbuf)
{
    return (bool)(p_rbuf->in == p_rbuf->out);
}


bool ringbuf_isfull(struct ringbuf *p_rbuf)
{
    int n = p_rbuf->in - p_rbuf->out + 1;

    return (bool)((n == 0) || (n == p_rbuf->size));
}


uint32_t ringbuf_freebytes(struct ringbuf *p_rbuf)
{
    int n = p_rbuf->out - p_rbuf->in - 1;

    if (n < 0) {
        n += p_rbuf->size;
    }

    return (n);
}


uint32_t ringbuf_nbytes(struct ringbuf *p_rbuf)
{
    int n = p_rbuf->in - p_rbuf->out;

    if (n < 0) {
        n += p_rbuf->size;
    }

    return (n);
}


void ringbuf_put_ahead(struct ringbuf *p_rbuf, char byte, uint32_t offset)
{
    int n = p_rbuf->in + offset;

    if (n >= p_rbuf->size) {
        n -= p_rbuf->size;
    }

    *(p_rbuf->buf + n) = byte;
}


void ringbuf_move_ahead(struct ringbuf *p_rbuf, uint32_t n)
{
    n += p_rbuf->in;

    if (n >= p_rbuf->size) {
        n -= p_rbuf->size;
    }

    p_rbuf->in = n;
}

/* end of file */
