#ifndef CONST_HPP
#define CONST_HPP

#include "main.h"

#ifndef PI
#define PI 3.14159265358979323846 // Hang so pi
#endif

#define DUONGKINHBANH 0.06             // MET Duong kinh banh xe
#define CHUVIBANH (DUONGKINHBANH * PI) // MET Chu vi banh xe
#define XUNG1VONG 40                   // XUNG So xung 1 vong quay theo ly thuyet 20 xung và đêm 2 suon

#define lx 0.0855f    // MÉT một nửa chiều rộng đế robot, tâm 2 bánh khác phía (gồm độ dày bánh xe)
#define ly 0.0575f    // MÉT một nửa chiều dài đế robot, tâm 2 bánh cùng phía
#define lxy (lx + ly) // lxy = lx + ly

#define MET1XUNG (CHUVIBANH / XUNG1VONG) // MET So met tuong duong 1 xung
#define XUNG1MET (1.0f / MET1XUNG)       // XUNG So xung tuong duong 1 met

const float ALPHA = 0.01;     // He so de tang/giam toc tu tu cho dong co
const float delta_ccr = 2.0f; // Khi ccrHT gan bang ccrTL, cho ccrHT = ccrTL

#define TIMER_10MS 10
#define TIMER_20MS 20
#define TIMER_50MS 50
#define TIMER_100MS 100
#define TIMER_200MS 200
#define TIMER_500MS 500
#define TIMER_1000MS 1000
#define TIMER_2000MS 2000
#define TIMER_5000MS 5000

// Hàm map đa năng, an toàn, dùng được cho cả float và int
template <typename T>
T doi_van_toc(T _vx, T in_vx_min, T in_vx_max, T out_ccr_min, T out_ccr_max)
{
    // Chốt an toàn: Chống lỗi chia cho 0 gây treo chip STM32
    if (in_vx_min == in_vx_max)
    {
        return out_ccr_min;
    }

    return (_vx - in_vx_min) * (out_ccr_max - out_ccr_min) / (in_vx_max - in_vx_min) + out_ccr_min;
}

#endif