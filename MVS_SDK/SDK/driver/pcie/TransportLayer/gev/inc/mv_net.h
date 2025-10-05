#ifndef __MV_NET_H__
#define __MV_NET_H__

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>
#include <linux/if_ether.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/tcp.h>
#include <linux/udp.h>

#define MAC_ADDR_LEN                            6
#define MV_NETDEV_PRIV(dev)                netdev_priv(dev)

typedef struct _MV_POINTER_CONTAINER_
{
    void*          pPrivate;
}MV_POINTER_CONTAINER, *PMV_POINTER_CONTAINER;

typedef struct _MV_NET_DEVICE_
{
    void*           pNetDevice;
    void*           pstPort;        //�������豸������port
    unsigned char   chMacAddr[MAC_ADDR_LEN];
}MV_NET_DEVICE, *PMV_NET_DEVICE;

typedef struct _MV_SK_BUFFER_
{
    void*               pSkBuffer;
    unsigned char*      pBuffer;    //����Э��ͷ��buffer
    unsigned int        nLen;       //����Э��ͷ��buffer����
    void*               pNetDevice; //�󶨵��ں�net_devָ��
}MV_SK_BUFFER, *PMV_SK_BUFFER;

/*�޸������豸��mtu����*/
int MV_NetDeviceChangeMTU(struct net_device* dev, int new_mtu);
/*��ȡ�����豸��link״̬*/
unsigned int MV_GetLink(struct net_device* dev);
extern unsigned int NetGetLink(void* pstPrivate);
/*���������豸*/
extern int MV_AllocNetDev(PMV_NET_DEVICE pstNetDevice, char* chDeviceName);
/*���������豸*/
extern void MV_FreeNetDev(PMV_NET_DEVICE pstNetDevice);
/*���ں�ע�������豸*/
extern int MV_RegisterNetDev(PMV_NET_DEVICE pstNetDevice);
/*���ں˽�ע�������豸*/
extern void MV_UnRegisterNetDev(PMV_NET_DEVICE pstNetDevice);
/*֪ͨ�ں˼�����·*/
extern void MV_NetifCarrierOn(PMV_NET_DEVICE pstNetDevice);      
/*֪ͨ�ں���·ʧЧ*/
extern void MV_NetifCarrierOff(PMV_NET_DEVICE pstNetDevice);  

/*����*/
/*�����ص�������Э��ջ����������*/
int MV_NetDeviceSendPkt(struct sk_buff* skb, struct net_device* dev);
extern void NetDeviceSendPkt(void* pSendBuffer, unsigned int nSendLen, void* pSkBuffer, void* pstPrivate);
/*��ֹ�ϲ�������������������ݰ�*/
extern void MV_NetifStopQueue(PMV_NET_DEVICE pstNetDevice);    
/*֪ͨ�ϲ�������������������ݰ�*/
extern void MV_NetifStartQueue(PMV_NET_DEVICE pstNetDevice);    
/*�ͷŷ���sk_buff������*/
extern void MV_DevKfreeSkb(PMV_SK_BUFFER pstSkBuffer);    
/*���ݷ�����ɣ�������������*/
extern void MV_NetifWakeQueue(PMV_NET_DEVICE pstNetDevice);    
/*��ȡ��Ч�غ�buffer*/
extern unsigned char* MV_GetSkbPayload(PMV_SK_BUFFER pstSkBuffer);   

/*�հ�*/
/*�ں˷���sk_buff*/
extern int MV_DevAllocSkb(PMV_SK_BUFFER pstSkBuffer, unsigned int nLen);
/*��ȡ�����ͷ�����ȣ�֧�ֽ���ipv4,ipv6�����������ͷ��ز�֧��*/
extern int MV_GetHeaderLen(unsigned char* pRxBuffer, unsigned int* nHeadLen);
/*ע��sk_buff�������ص�*/
extern void MV_SkbDestructor(struct sk_buff* skb);
extern int MV_SetSkbDestructor(PMV_SK_BUFFER pstSkBuffer, void (*destructor)(struct sk_buff *skb));
/*����sk_buff�������հ����*/
extern void NetResetRxNode(void* pstPrivate);
/*����sk_buff��˽��ָ��*/
extern int MV_SetSkbPrivate(PMV_SK_BUFFER pstSkBuffer, void* pstPrivate);
/*��ȡsk_buff��˽��ָ��*/
extern void* MV_GetSkbPrivate(struct sk_buff *skb);
/*sk_buff����������*/
extern int MV_SkbPut(PMV_SK_BUFFER pstSkBuffer, unsigned char* pRxBuffer, unsigned int nLen);
/*sk_buff������������*/
extern void MV_SkbAddRxFrag(PMV_SK_BUFFER pstSkBuffer, void* pstPage, unsigned int nPageOffset, unsigned int nBlockSize);
extern int MV_NetifRx(PMV_SK_BUFFER pstSkBuffer);
#endif  // __MV_NET_H__
