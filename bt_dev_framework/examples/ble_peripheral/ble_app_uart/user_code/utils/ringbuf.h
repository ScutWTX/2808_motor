#ifndef __RINGBUF_H
#define __RINGBUF_H
 
#include <stdbool.h>
#include <stdint.h>

#define RINGBUF_OK             0
#define RINGBUF_INVALID_PARAM -1


/**
 * \brief ���λ������ṹ��
 */
struct ringbuf {
    int      in;     /**< \brief д��� */
    int      out;    /**< \brief ��ȡ�� */
    int      size;   /**< \brief ��������С */
    uint8_t *buf;    /**< \brief ����������ָ�� */
};

int ringbuf_init(struct ringbuf *p_rbuf, uint8_t *p_buf, uint32_t size);

int ringbuf_putchar(struct ringbuf *p_rbuf, const char data);
int ringbuf_getchar(struct ringbuf *p_rbuf, uint8_t *p_data);

uint32_t ringbuf_put(struct ringbuf *p_rbuf, const uint8_t *p_buf, uint32_t nbytes);
uint32_t ringbuf_get(struct ringbuf *p_rbuf, uint8_t *p_buf, uint32_t nbytes);

uint32_t ringbuf_put_with_length(struct ringbuf *p_rbuf, const uint8_t *p_buf, uint32_t nbytes);
uint32_t ringbuf_get_with_length(struct ringbuf *p_rbuf, uint8_t *p_buf, uint32_t *p_nbytes);

void ringbuf_flush(struct ringbuf *p_rbuf);
bool ringbuf_isempty(struct ringbuf *p_rbuf);
bool ringbuf_isfull(struct ringbuf *p_rbuf);
uint32_t ringbuf_freebytes(struct ringbuf *p_rbuf);
uint32_t ringbuf_nbytes(struct ringbuf *p_rbuf);
 
#endif /* __RINGBUF_H */
