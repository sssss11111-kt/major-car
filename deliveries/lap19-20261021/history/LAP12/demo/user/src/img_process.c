/*
 * 逐飞 RT1064 摄像头组：赛道边线与中线提取
 * 原版：第十九届东南大学智能车竞赛基础四轮组开源学习代码（Otsu + 全图二值化 + 固定中点扫线）
 * 本版：在每个接口内部替换算法，函数名和参数保持不变，main.c 不需要改。
 *
 * 改动要点（对应"弯道丢边停车"）：
 *  1. 阈值按行带估算：只用一个全图阈值时，赛道远端+地板+反光混在一起，
 *     地板上被当成赛道或赛道被当成地板，弯道里边界直接消失。现在把图像纵向分成若干带，
 *     每带单独直方图求阈值，并加上"至少比本带最暗像素高 X%"的下限，防止全白反光带把阈值推到 250+。
 *  2. 从画面底部开始向上逐行找最大白色连通段，不再从固定 x=94 往两边撞。
 *     弯道里赛道中心已经跑到 x=40，固定中点扫线只会扫到地板。
 *  3. 单边也出中线：只有一个边界时用本行半宽补另一边（半宽来自双边行的滑动平均，
 *     并且用透视模型给出初值），不再要求左右都检出。
 *  4. 有效性改为多行统计（可用行数 / 双边行数 / 底部行是否可信），不再只看第 90 行一个点。
 *  5. 输出 img_get_valid()、img_get_error() 给主循环用：一个平滑后的"赛道在画面里偏了多少像素"。
 *
 * 说明：所有阈值常量都是"先在低速、单一光照下标定"的初值，不要把这里的数字当成普适最优值。
 */

#include "img_process.h"
#include "string.h"

/* 不用 abs()：Keil 下 <stdlib.h> 的 abs 声明容易和其它头冲突，MSVC/ARMCC 行为也不同 */
static int ip_abs(int v) { return v < 0 ? -v : v; }

/* ---------- 可调常量 ---------- */
#define IP_W              (MT9V03X_W)
#define IP_H              (MT9V03X_H)
#define IP_BANDS          8          /* 纵向分带数，每带 IP_H/IP_BANDS 行 */
#define IP_BAND_MIN_ROWS  8          /* 少于此行数的带直接沿用相邻带阈值 */
#define IP_LOW_MARGIN_PCT 22         /* 阈值下限 = 带内最暗 + (最亮-最暗)*22/100 */
#define IP_MIN_DR          10        /* 一帧内最亮-最暗低于这个值，视为无有效场景 */

/* 场景对比度兜底：全黑/全白/只有均匀灰的帧，动态范围很小，直接判无效。 */
static uint8_t s_scene_ok;
#define IP_TRACK_TOP      18         /* 只处理 y>=IP_TRACK_TOP 的行（更远的行噪声大、透视太扁） */
#define IP_TRACK_BOTTOM   (IP_H-1)   /* 从这一行开始向上扫 */
#define IP_RUN_MIN        10         /* 白色连通段最少像素，短于此视为噪点 */
#define IP_RUN_GOOD       24         /* 长度达到这个值，就优先选"离参考中心最近"的段 */
#define IP_JUMP_MAX       22         /* 相邻两行中线允许的最大跳变（像素） */
#define IP_BOTH_SIDE_MIN  3          /* 一帧里至少有这么多行左右都检出，才更新半宽表 */
#define IP_VALID_ROWS_MIN 30         /* 至少这么多行可用，才算这一帧有效 */
#define IP_HALF_PERSPECTIVE_SPAN 32.0f /* Keep the original per-row perspective slope. */
#define IP_HALF_ALPHA     4          /* 半宽平滑：新值权重 1/IP_HALF_ALPHA */
#define IP_ERROR_ROW_TOP  (IP_H-30)  /* 误差取这一行到画面底部的加权平均 */
#define IP_ERROR_ROW_BOT  (IP_H-2)
#define IP_ERROR_NEAR     8          /* 越靠近底部权重越大：这一行更重要 */

/* ---------- 帧间状态（函数内部用，不暴露给 main.c） ---------- */
static uint8_t  s_half[IP_H];        /* 每行赛道半宽，0 = 未标定 */
static int16_t  s_last_center[IP_H]; /* 上一帧中线，用于跳变限制 */
static uint8_t  s_row_thr[IP_H];     /* 本帧逐行阈值，供"中线必须是赛道"的检查用 */
static uint8_t  s_half_ready;        /* 半宽表是否已经由真实双边行建立 */
static int16_t  s_half_bias;         /* 视野下半部的半宽误差：模型宽度与实测宽度的固定偏差 */
static uint8_t  s_half_bias_ready;
static uint8_t  s_have_error;        /* 是否已经有可用的 s_error（初值 0 不能当历史误差做限幅） */
static uint8_t  s_valid;             /* 本帧是否有效 */
static int16_t  s_error;             /* 本帧误差（像素）：正 = 赛道中心在画面中心右侧 */
static uint8_t  s_usable_rows;       /* 本帧可用行数（调试/遥测用） */
static uint8_t  s_both_rows;         /* 本帧双边行数 */

/* ---------- 小工具 ---------- */
static int ip_clamp(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* 透视初值：上半窄、下半宽，线性插值。真实安装角度一旦改变，用直道重新标定。 */
static int ip_half_default(int y)
{
    float offset = (float)(y - IP_WIDTH_CAL_ROW) / (float)(IP_H - 1);
    return (int)(IP_WIDTH_CAL_PX / 2.0f + IP_HALF_PERSPECTIVE_SPAN * offset + 0.5f);
}

/* Inclusive tolerance: a difference of exactly 15 full-width pixels is valid. */
static int ip_width_matches(int y, int width)
{
    return ip_abs(width - 2 * ip_half_default(y)) <= IP_WIDTH_TOLERANCE_PX;
}

static int ip_half(int y)
{
    int v;
    if (y < 0 || y >= IP_H) return ip_half_default(0);
    v = s_half[y] ? s_half[y] : ip_half_default(y);
    /* 标定过的常值偏差：摄像头高度/俯仰角与透视初值不一致时，用一个整体偏移兜住 */
    v += s_half_bias;
    if (v < 12) v = 12;
    if (v > IP_W / 2) v = IP_W / 2;
    return v;
}

/* 行带阈值：hist 必须是"该带所有行的灰度直方图"。 */
static uint8_t ip_band_threshold(const uint32_t *hist, uint32_t total)
{
    int lo = 0, hi = 255;
    uint32_t acc = 0, sum = 0;
    double best = -1.0, back = 0.0;
    uint32_t w = 0;
    uint8_t thr = 100;

    if (!total) return 100;
    while (lo < 255 && !hist[lo]) ++lo;
    while (hi > lo && !hist[hi]) --hi;
    if (hi <= lo) return (uint8_t)hi;             /* 整带只有一个灰度值 */

    for (int i = lo; i <= hi; ++i) sum += (uint32_t)i * hist[i];
    for (int i = lo; i < hi; ++i) {
        w += hist[i];
        if (!w || w == total) continue;
        back += (double)i * hist[i];
        double score = (double)w * (double)(total - w) *
                       (back / w - (double)(sum - (uint32_t)back) / (double)(total - w)) *
                       (back / w - (double)(sum - (uint32_t)back) / (double)(total - w));
        if (score > best) { best = score; thr = (uint8_t)i; }
    }
    /* 关键保护：阈值不能低于"本带最暗 + 动态范围的一部分"，
       否则一条全是赛道的带（没有地板）会退化成 100 左右，把整带判成白，
       或者一条全是反光的带会把阈值推到 240+，地板全黑、赛道也断。 */
    {
        int floor_thr = lo + (hi - lo) * IP_LOW_MARGIN_PCT / 100;
        if (thr < floor_thr) thr = (uint8_t)ip_clamp(floor_thr, lo, 254);
    }
    (void)acc;
    return thr;
}

/* 在当前行找白色连通段中"最可能是赛道"的那一段。
 * 评分不只看长度：左侧地板和赛道一样白（反光）时，"最长段"会连成一整片，
 * 所以这里把"预测的左边界位置"和"预期半宽"一起作为约束。
 *   pred_left : 由上一行/上一帧推算出的左边界，<0 表示没有
 *   expect_hw : 该行的预期半宽
 *   left_ok/right_ok : 输出：该边界是否在画面内（贴边说明已出画，不可信）
 * 返回 1 表示找到。 */
static int ip_row_run(const uint8_t *bin, const uint8_t *src, int thr,
                      int pred_left, int expect_hw, int ref_center,
                      int *left, int *right)
{
    int best_l = -1, best_r = -1, best_score = -1000000, best_d = 0x7fff;
    int max_wide = (expect_hw * 2) + (expect_hw / 2) + 12;  /* 允许赛道宽 1.75 倍 */
    int x = 0;
    (void)thr;

    while (x < IP_W) {
        int l, r, len, score, d, wide, dev;
        while (x < IP_W && !bin[x]) ++x;
        if (x >= IP_W) break;
        l = x;
        while (x < IP_W && bin[x]) ++x;
        r = x - 1;
        len = r - l + 1;
        if (len < IP_RUN_MIN) continue;

        /* 反光把赛道 + 地板连成一片时，先按灰度台阶把它切开 */
        if (len > max_wide) {
            int cut = -1, best = 24, side = 0;   /* side: 1=切左, 2=切右 */
            for (int k = l + 10; k < r - 6; ++k) {
                int dl = (int)src[k] - (int)src[k - 2];
                int dr = (int)src[k] - (int)src[k + 2];
                int d0 = dl > dr ? dl : dr;
                if (d0 > best) { best = d0; cut = k; side = (dl >= dr) ? 1 : 2; }
            }
            if (cut > 0) {
                if (side == 1) { l = cut; } else { r = cut; }
                len = r - l + 1;
                if (len < IP_RUN_MIN) continue;
            }
        }
        wide = (len > 150) ? 150 : len;
        d    = ((l + r) / 2) - ref_center;
        if (d < 0) d = -d;
        /* 半宽偏差：期望值越大，允许的绝对偏差也越大 */
        dev = (wide / 2) - expect_hw;
        if (dev < 0) dev = -dev;
        score = wide * 4 - d - dev * 5;
        if (pred_left >= 0) {
            int e = l - pred_left;
            if (e < 0) e = -e;
            score -= e * 3;
        }
        /* 同分时选离参考中心更近的段（反光段通常更靠外） */
        if (score > best_score || (score == best_score && d < best_d)) {
            best_score = score; best_d = d; best_l = l; best_r = r;
        }
    }
    if (best_l < 0) return 0;
    *left = best_l;
    *right = best_r;
    return 1;
}

/* ---------- 对外接口 ---------- */

/* 原来的全图二值化保留，但 main.c 已不再依赖它作为扫线输入；
 * 若还想在屏幕上显示二值图，可以用全局 Otsu 的结果调它。 */
void binaryzation_process(unsigned char *_img, const unsigned short _rows,
                          const unsigned short _cols, const unsigned int _threshold_value)
{
    unsigned int size = (unsigned int)_rows * _cols;
    for (unsigned int i = 0; i < size; ++i)
        _img[i] = (_img[i] > _threshold_value) ? 255 : 0;
}

void get_hist_gram(unsigned char *_img, const unsigned short _rows,
                   const unsigned short _cols, short *_hist_gram)
{
    memset(_hist_gram, 0, 256 * sizeof(short));
    for (unsigned int i = 0; i < (unsigned int)_rows * _cols; ++i)
        ++_hist_gram[_img[i]];
}

/* 全图 Otsu：现在只用于屏幕显示和遥测上报，不再决定扫线边界。 */
unsigned char get_threshold_otsu(const short *_hist_gram)
{
    uint32_t hist[256];
    uint32_t total = 0;
    for (int i = 0; i < 256; ++i) { hist[i] = (uint32_t)(_hist_gram[i] < 0 ? 0 : _hist_gram[i]); total += hist[i]; }
    return ip_band_threshold(hist, total);
}

void img_process_reset(void)
{
    memset(s_half, 0, sizeof s_half);
    memset(s_last_center, 0xff, sizeof s_last_center); /* 全 -1 */
    s_half_ready = 0;
    s_half_bias = 0;
    s_half_bias_ready = 0;
    s_have_error = 0;
    s_scene_ok = 0;
    s_valid = 0;
    s_error = 0;
    s_usable_rows = 0;
    s_both_rows = 0;
}

uint8_t  img_get_valid(void)      { return s_valid; }
int16_t  img_get_error(void)      { return s_error; }
uint8_t  img_get_usable_rows(void){ return s_usable_rows; }
uint8_t  img_get_both_rows(void)  { return s_both_rows; }

/*
 * 兼容原接口：main.c 现在仍然写
 *   auxiliary_process((uint8_t *)image, MT9V03X_H, MT9V03X_W, 127, left_line, mid_line, right_line);
 * 但"image"必须传入原始灰度图（不要先做全图二值化），_threshold_val 已不使用，
 * 内部自己按行带算阈值。返回的 left/mid/right 用与原版相同的约定：
 *   left  = 左边界像素；检不到时为 0
 *   right = 右边界像素；检不到时为 MT9V03X_W-1
 *   mid   = 中线像素；该行不可用时为 0
 * 这样 main.c 里"image[row][mid_line[row]]==255"这类判断仍然能编译、能工作。
 */
void auxiliary_process(uint8_t *_src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols,
                       unsigned char _threshold_val,
                       uint8_t *_left_line, uint8_t *_mid_line, uint8_t *_right_line)
{
    static uint32_t band_hist[256];             /* 1KB，按带复用 */
    uint8_t  band_thr[IP_BANDS];
    uint8_t  row_thr[IP_H];
    uint8_t  bin[IP_W];
    int      band_rows = (IP_H + IP_BANDS - 1) / IP_BANDS;
    int      seed, y, usable = 0, both = 0;
    long     err_sum = 0;
    int      err_n = 0;

    (void)_threshold_val;
    if (_src_rows != IP_H || _src_cols != IP_W) return;
    if (!_src_pixel_mat || !_left_line || !_mid_line || !_right_line) return;

    /* ---- 1. 按行带求阈值 ----
       同一块 256 直方图重复使用，避免 8 带 × 256 × 4B = 8KB 的静态占用。 */
    for (int b = 0; b < IP_BANDS; ++b) {
        uint32_t total = 0;
        int y0 = b * band_rows;
        int y1 = (y0 + band_rows < IP_H) ? (y0 + band_rows) : IP_H;
        memset(band_hist, 0, sizeof band_hist);
        for (y = y0; y < y1; ++y) {
            const uint8_t *src = _src_pixel_mat + (unsigned)y * IP_W;
            for (int x = 0; x < IP_W; ++x) ++band_hist[src[x]];
        }
        for (int i = 0; i < 256; ++i) total += band_hist[i];
        if (total < (uint32_t)IP_BAND_MIN_ROWS * IP_W)
            band_thr[b] = b ? band_thr[b - 1] : 128;   /* 带太小/全空：沿用上一带 */
        else
            band_thr[b] = ip_band_threshold(band_hist, total);
        s_scene_ok = 0;
    }
    for (y = 0; y < IP_H; ++y) { row_thr[y] = band_thr[y / band_rows]; s_row_thr[y] = row_thr[y]; }
    {
        int lo = 255, hi = 0;
        for (y = 0; y < IP_H; ++y) {
            const uint8_t *src = _src_pixel_mat + (unsigned)y * IP_W;
            for (int x = 0; x < IP_W; ++x) { if (src[x] < lo) lo = src[x]; if (src[x] > hi) hi = src[x]; }
        }
        s_scene_ok = (uint8_t)(hi - lo >= IP_MIN_DR);
    }

    /* ---- 2. 从底部向上逐行找边界 ---- */
    for (y = 0; y < IP_H; ++y) { _left_line[y] = 0; _right_line[y] = MT9V03X_W - 1; _mid_line[y] = 0; }

    /* 第一遍只测双边行，用来给本帧的单边补线提供半宽。
       固定透视初值只能当兜底，真实半宽必须来自本帧/历史双边行。 */
    for (y = IP_TRACK_BOTTOM; y >= IP_TRACK_TOP; --y) {
        const uint8_t *src = _src_pixel_mat + (unsigned)y * IP_W;
        uint8_t thr = row_thr[y];
        int l, r, width;
        int ref = (y < IP_H - 1 && s_last_center[y + 1] >= 0) ? s_last_center[y + 1] : IP_W / 2;

        for (int x = 0; x < IP_W; ++x) bin[x] = (uint8_t)(src[x] > thr);
        if (!ip_row_run(bin, src, thr, -1, ip_half(y), ref, &l, &r)) continue;
        width = r - l + 1;
        if (l <= 2 || r >= IP_W - 3) continue;          /* 只有两边都在画面里才可信 */
        if (width < 30 || width > IP_W - 20) continue;
        if (!ip_width_matches(y, width)) continue;     /* Reject unreliable width-calibration samples. */
        {
            int old = s_half[y] ? s_half[y] : ip_half_default(y);
            s_half[y] = (uint8_t)((old * (IP_HALF_ALPHA - 1) + width / 2) / IP_HALF_ALPHA);
            ++both;
        }
    }
    if (both >= IP_BOTH_SIDE_MIN) s_half_ready = 1;
    /* 下半部（y >= H-40）双边行：把"模型半宽"和"实测半宽"的差累计成一个整体偏差，
       只有样本够多时才更新，避免弯道里被一两行带偏。 */
    {
        long diff_sum = 0;
        int  diff_n = 0;
        for (y = IP_H - 40; y < IP_H; ++y) {
            int m = ip_half(y), w = s_half[y];
            if (!s_half_bias_ready) m = ip_half_default(y);   /* 还没有偏差时直接用模型 */
            if (w >= 20) { diff_sum += ((int)w) - m; ++diff_n; }
        }
        if (diff_n >= 8) {
            int bias = (int)(diff_sum / diff_n);
            bias = (bias < -20) ? -20 : ((bias > 20) ? 20 : bias);
            if (s_half_bias_ready) s_half_bias = (int16_t)((s_half_bias * 3 + bias) / 4);
            else { s_half_bias = (int16_t)bias; s_half_bias_ready = 1; }
        }
    }

    /* 第二遍：正式逐行追踪，单边补线使用上一步更新过的半宽 */
    seed = IP_W / 2;
    for (y = IP_TRACK_BOTTOM; y >= IP_TRACK_TOP; --y) {
        const uint8_t *src = _src_pixel_mat + (unsigned)y * IP_W;
        uint8_t thr = row_thr[y];
        int l, r, width, center, half;
        int ref;

        for (int x = 0; x < IP_W; ++x) bin[x] = (uint8_t)(src[x] > thr);

        ref = (y < IP_H - 1 && s_last_center[y + 1] >= 0) ? s_last_center[y + 1] : seed;
        {
            int pred_left = ref - ip_half(y);
            if (!ip_row_run(bin, src, thr, pred_left, ip_half(y), ref, &l, &r)) continue;
        }
        if (r - l + 1 < IP_RUN_MIN) continue;

        width = r - l + 1;
        /* 段贴到画面右侧：右边界已经跑出视野，右边界不可信 */
        {
            int left_ok  = (l > 2);
            int right_ok = (r < IP_W - 3);
            /* 段的一侧是"反光连成片"时，二分图里看不出来，但灰度里有台阶：
               若右边界已贴边，且段内存在一个明显台阶（向右陡降），把右边界收到台阶处。 */
            if (!right_ok) {
                int k, cut = -1, best = 24;
                for (k = r - 6; k > l + 8; --k) {
                    int drop = (int)src[k] - (int)src[k + 2];
                    if (drop > best) { best = drop; cut = k; }
                }
                if (cut > 0 && cut < r - 6) { r = cut; right_ok = 1; }
            }
            if (!left_ok) {
                int k, cut = -1, best = 24;
                for (k = l + 6; k < r - 8; ++k) {
                    int drop = (int)src[k] - (int)src[k - 2];
                    if (drop > best) { best = drop; cut = k; }
                }
                if (cut > 0 && cut > l + 6) { l = cut; left_ok = 1; }
            }
            /* 但画面两侧都可能本来就有白色地板/反光，长度超过中位宽度 1.6 倍的段，
               边界更可能是噪声而不是赛道边 */
            int mid = (l + r) / 2;
            half = ip_half(y);
            if (width > half * 2 + half) {
                /* 段太长：说明白地板连成一整片，用参考中心重新切一段 */
                l = ip_clamp(mid - half, 1, IP_W - 2);
                r = ip_clamp(mid + half, 1, IP_W - 2);
                width = r - l + 1;
                left_ok  = (l > 2);
                right_ok = (r < IP_W - 3);
            }
            if (!left_ok && !right_ok) continue;

            if (left_ok && right_ok) {
                center = mid;                            /* 双边都可信 */
            } else if (left_ok) {
                center = l + half;               /* 只有左边：右边补半宽 */
            } else {
                center = r - half;               /* 只有右边：左边补半宽 */
            }
        }

        if (center < 2 || center >= IP_W - 2) continue;
        if (s_last_center[y] >= 0 && ip_abs(center - s_last_center[y]) > IP_JUMP_MAX) {
            /* 与上一帧同一行差太多：多半是反光/阴影误判，这一行不采信 */
            continue;
        }
        _left_line[y]  = (uint8_t)l;
        _right_line[y] = (uint8_t)r;
        _mid_line[y]   = (uint8_t)center;
        ++usable;
        seed = center;
        if (y >= IP_ERROR_ROW_TOP && y <= IP_ERROR_ROW_BOT) {
            /* 越靠近车头权重越大，避免远处几行把小车的实际位置带偏 */
            int wgt = IP_ERROR_NEAR + (y - IP_ERROR_ROW_TOP);
            err_sum += (long)center * wgt;
            err_n += wgt;
        }
    }

    /* ---- 3. 本帧统计与有效性 ---- */
    s_usable_rows = (uint8_t)(usable > 255 ? 255 : usable);
    s_both_rows   = (uint8_t)(both > 255 ? 255 : both);

    s_valid = (uint8_t)(s_scene_ok && usable >= IP_VALID_ROWS_MIN);
    if (s_valid) {
        int e = err_n ? (int)(err_sum / err_n) : (IP_W / 2);
        e -= IP_W / 2;
        /* 一阶限幅，避免单帧跳动直接打到舵机；只有拿到过有效误差后才做限幅 */
        if (s_have_error) {
            if (e >  s_error + 16) e = s_error + 16;
            if (e <  s_error - 16) e = s_error - 16;
        }
        /* 死区：|e|<=2px 视为居中，避免直道上左右微抖 */
        if (e > -3 && e < 3) e = 0;
        s_error = (int16_t)ip_clamp(e, -IP_W / 2, IP_W / 2);
        s_have_error = 1;
    }

    /* ---- 3c. 把中线吸附到真正属于赛道(白)的像素上 ----
       单边补线算出来的是估计值，可能落在边界外 1~2 个像素；
       而 main.c 原来用 image[row][mid]==255 判断，偶尔会因此判无效。
       这里在 ±3 像素内找第一个满足"该灰度 > 本行阈值"的点。 */
    for (y = IP_TRACK_TOP; y < IP_H; ++y) {
        const uint8_t *src = _src_pixel_mat + (unsigned)y * IP_W;
        int m = _mid_line[y], k, chosen = -1;
        if (!m) continue;
        if (src[m] > s_row_thr[y]) continue;
        for (k = 1; k <= 3 && chosen < 0; ++k) {
            if (m - k >= 1 && src[m - k] > s_row_thr[y]) chosen = m - k;
            else if (m + k < IP_W - 1 && src[m + k] > s_row_thr[y]) chosen = m + k;
        }
        if (chosen >= 0) { _mid_line[y] = (uint8_t)chosen; m = chosen; }
        else {
            /* ±3 内都找不到赛道：这一行不该被当真，退回上一帧或相邻行 */
            _mid_line[y] = 0;
        }
        if (_mid_line[y]) {
            int half = ip_half(y);
            if (_left_line[y])  _left_line[y]  = _mid_line[y] > half ? (uint8_t)(_mid_line[y] - half) : 0;
            if (_right_line[y]) _right_line[y] = _mid_line[y] + half < IP_W ? (uint8_t)(_mid_line[y] + half) : IP_W - 1;
        }
    }

    /* ---- 3b. 空洞填补：相邻行可用就插值，本帧不可用就沿用上一帧 ----
       目的：main.c 里 mid_line[90] 是唯一参与打角的量，不能出现 0
       （0 会被当成"赛道在最左边"，舵机打满）。 */
    for (y = IP_H - 1; y >= 0; --y) {
        int up, down;
        if (_mid_line[y]) continue;
        up = down = -1;
        for (int k = y + 1; k < IP_H; ++k) if (_mid_line[k]) { up = k; break; }
        for (int k = y - 1; k >= 0; --k)    if (_mid_line[k]) { down = k; break; }
        if (up >= 0 && down >= 0) {
            int a = _mid_line[up], b = _mid_line[down];
            _mid_line[y] = (uint8_t)((a * (y - down) + b * (up - y)) / (up - down));
            _left_line[y]  = _mid_line[y] > ip_half(y) ? (uint8_t)(_mid_line[y] - ip_half(y)) : 0;
            _right_line[y] = _mid_line[y] + ip_half(y) < IP_W ? (uint8_t)(_mid_line[y] + ip_half(y)) : IP_W - 1;
        } else if (s_last_center[y] >= 0) {
            _mid_line[y] = (uint8_t)s_last_center[y];
            _left_line[y]  = _mid_line[y] > ip_half(y) ? (uint8_t)(_mid_line[y] - ip_half(y)) : 0;
            _right_line[y] = _mid_line[y] + ip_half(y) < IP_W ? (uint8_t)(_mid_line[y] + ip_half(y)) : IP_W - 1;
        }
    }

    /* ---- 4. 记录本帧中线，供下一帧跳变限制 ---- */
    for (y = 0; y < IP_H; ++y)
        s_last_center[y] = _mid_line[y] ? (int16_t)_mid_line[y] : -1;
}
