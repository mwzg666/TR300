#ifndef CALCDOSERATE_H
#define CALCDOSERATE_H

#ifdef __cplusplus
extern  "C" {
#endif

#ifdef P2P_NH
// ��Ե��������
float CpsToUsv_h(NH_PARAM *Nh, float x);
#else
float CpsToUsv_h(float parama, float paramb, float paramc, float count);
#endif

#ifdef __cplusplus
}
#endif
#endif  
