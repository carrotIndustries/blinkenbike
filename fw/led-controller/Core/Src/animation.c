#include "animation.h"
#include "util.h"
#include "led.h"
#include <string.h>

static uint8_t animation_pattern_current = 0xff;

typedef struct {
    void (*render)(led_t *buf, uint8_t step, void *state, const void *conf);
    void (*reset)(void *state, const void *conf);
    const void *user_data;
    uint8_t speed_mul;
} animation_t;



typedef struct {
    uint16_t state;
} anim_state_single_color_chase_t;

typedef struct {
    led_t color;
    uint16_t mod;
    uint16_t width;
}  anim_conf_single_color_chase_t;

static void single_color_chase_render(led_t *buf, uint8_t steps, void *p_state, const void *p_conf)
{
    const anim_conf_single_color_chase_t *conf = p_conf;
    anim_state_single_color_chase_t *state = p_state;
    if(steps) {
        while(steps--) {
            state->state++;
            if(state->state == conf->mod)
            state->state = 0;
        }
    }
    /*for(uint16_t i = 0; i < N_LEDS_PER_SIDE; i++) {
        led_t v= {0,0,0};
        if(i >= state->state) {
            uint16_t x = (i- state->state)%conf->mod;
            if(x < conf->width)
                v = conf->color;
        }
        led_set_lr(buf, i, v);
    }*/
    for(uint16_t i = 0; i < N_LEDS_PER_SIDE+conf->mod; i++) {
        led_t v= {0,0,0};
        if(i >= state->state) {
            uint16_t x = (i- state->state)%conf->mod;
            if(x < conf->width)
                v = conf->color;
        }
        if(i >= conf->mod)
            led_set_lr(buf, i-conf->mod, v);
    }
}

static const anim_conf_single_color_chase_t single_color_chase_red1 = 
{
    .color = {0xff, 0, 0},
    .mod = 13,
    .width = 1
};

static const anim_conf_single_color_chase_t single_color_chase_red2 = 
{
    .color = {0xff, 0, 0},
    .mod = N_LEDS_PER_SIDE,
    .width = 10
};




typedef struct {
    uint16_t state;
    uint16_t color_state;
} anim_state_multi_color_chase_t;

typedef struct {
    const led_t *colors;
    uint8_t n_colors;
    uint16_t mod;
    uint16_t width;
}  anim_conf_multi_color_chase_t;

static void multi_color_chase_render(led_t *buf, uint8_t steps, void *p_state, const void *p_conf)
{
    const anim_conf_multi_color_chase_t *conf = p_conf;
    anim_state_multi_color_chase_t *state = p_state;
    if(steps) {
        while(steps--) {
            state->state++;
            if(state->state == conf->mod) {
                state->state = 0;
                if(state->color_state == 0)
                    state->color_state = conf->n_colors-1;
                else
                    state->color_state--;
            }
        }
    }
    
    for(uint16_t i = 0; i < N_LEDS_PER_SIDE+conf->mod; i++) {
        led_t v= {0,0,0};
        if(i >= state->state) {
            uint16_t x = (i- state->state)%conf->mod;
            if(x < conf->width)
                v = conf->colors[((i- state->state)/conf->mod + state->color_state)%conf->n_colors];
        }
        if(i >= conf->mod)
            led_set_lr(buf, i-conf->mod, v);
    }
}

static const led_t color_off = {0,0,0};

static const led_t colors_rgb[] = {{.r=0xff}, {.g=0xff}, {.b=0xff}};

static const anim_conf_multi_color_chase_t multi_color_chase1 = 
{
    .colors = colors_rgb,
    .n_colors = ARRAY_SIZE(colors_rgb),
    .mod = N_LEDS_PER_SIDE,
    .width = 1
};

static const anim_conf_multi_color_chase_t multi_color_chase2 = 
{
    .colors = colors_rgb,
    .n_colors = ARRAY_SIZE(colors_rgb),
    .mod = 26,
    .width = 26
};

static const anim_conf_multi_color_chase_t multi_color_chase3 = 
{
    .colors = colors_rgb,
    .n_colors = ARRAY_SIZE(colors_rgb),
    .mod = N_LEDS_PER_SIDE/2,
    .width = N_LEDS_PER_SIDE/2
};

static const led_t colors_blue_yellow[] = {{.r=0xff, .g=0xff}, {.b=0xff}};

static const anim_conf_multi_color_chase_t multi_color_chase4 = 
{
    .colors = colors_blue_yellow,
    .n_colors = ARRAY_SIZE(colors_blue_yellow),
    .mod = 13,
    .width = 13
};

static const led_t colors_rainbow[] = {{.r=0xff}, {.r=0xff, .g=0x80}, {.r=0xff, .g=0xff}, {.g=0xff}, {.b=0xff}, {.b=0xff, .r=0xff}};


static const anim_conf_multi_color_chase_t multi_color_chase_rainbow = 
{
    .colors = colors_rainbow,
    .n_colors = ARRAY_SIZE(colors_rainbow),
    .mod = 13,
    .width = 13
};

typedef struct {
    uint32_t state;
} anim_state_prbs_t;

typedef struct {
    led_t color;
    uint32_t poly;
}  anim_conf_single_color_prbs_t;

static void single_color_prbs_reset(void *p_state, const void *p_conf)
{
    anim_state_prbs_t *state = p_state;
    state->state = 1;
}

static uint32_t prbs_advance(uint32_t value, uint32_t poly)
{
    uint32_t m = value&poly;
    uint32_t x = __builtin_parity(m);
    return (value << 1) | x;
}

static void single_color_prbs_render(led_t *buf, uint8_t steps, void *p_state, const void *p_conf)
{
    const anim_conf_single_color_prbs_t *conf = p_conf;
    anim_state_prbs_t *state = p_state;
    if(steps) {
        while(steps--) {
            state->state = prbs_advance(state->state, conf->poly);
        }
    }
    uint32_t reg = state->state;
    for(uint16_t i = 0; i < N_LEDS_PER_SIDE; i++) {    
        led_t v= {0,0,0};
        if(reg&1) {
            v = conf->color;
        }
        reg = prbs_advance(reg, conf->poly);
        led_set_lr(buf, N_LEDS_PER_SIDE-1-i, v);
    }
}

#define PRBS7 ((1<<6) | (1<<5))
#define PRBS15 ((1<<14) | (1<<13))

static const anim_conf_single_color_prbs_t single_color_prbs1 = 
{
    .color = {0xff, 0, 0xff},
    .poly = PRBS15
};



typedef struct {
    uint32_t poly;
}  anim_conf_rgb_prbs_t;

static void rgb_prbs_reset(void *p_state, const void *p_conf)
{
    anim_state_prbs_t *state = p_state;
    state->state = 1;
}

static void rgb_prbs_render(led_t *buf, uint8_t steps, void *p_state, const void *p_conf)
{
    const anim_conf_rgb_prbs_t *conf = p_conf;
    anim_state_prbs_t *state = p_state;
    if(steps) {
        while(steps--) {
            state->state = prbs_advance(state->state, conf->poly);
        }
    }
    uint32_t reg = state->state;
    for(uint16_t i = 0; i < N_LEDS_PER_SIDE; i++) {
        led_t v= {0,0,0};
        uint8_t brn = 0;
        if(reg&1)
            brn = 0xff;
        if(reg&(1<<20))
            v.r = brn;
        if(reg&(1<<21))
            v.g = brn;
        if(reg&(1<<22))
            v.b = brn;
        reg = prbs_advance(reg, conf->poly);
        led_set_lr(buf, N_LEDS_PER_SIDE-1-i, v);
    }
}

static const anim_conf_rgb_prbs_t rgb_prbs1 = 
{
    .poly = PRBS15
};

#define SIDE_RIGHT 1
#define SIDE_LEFT 2
#define SIDE_BOTH 3

typedef struct {
    led_t color;
    uint8_t sides;
}  anim_conf_single_color_fill_t;

static void single_color_fill_render(led_t *buf, uint8_t steps, void *p_state, const void *p_conf)
{
    const anim_conf_single_color_fill_t *conf = p_conf;
    for(uint16_t i = 0; i < N_LEDS_PER_SIDE; i++) {
        if(conf->sides == SIDE_BOTH)
            led_set_lr(buf, i, conf->color);
        else if(conf->sides == SIDE_LEFT) {
            led_set_l(buf, i, conf->color);
            led_set_r(buf, i, color_off);
        }
        else if(conf->sides == SIDE_RIGHT) {
            led_set_r(buf, i, conf->color);
            led_set_l(buf, i, color_off);
        }
    }
}

static const anim_conf_single_color_fill_t single_color_white_both = {
    .color = {.r = 0xff, .g=0xff, .b=0xff},
    .sides = SIDE_BOTH
};

static const anim_conf_single_color_fill_t single_color_white_left = {
    .color = {.r = 0xff, .g=0xff, .b=0xff},
    .sides = SIDE_LEFT
};

static const anim_conf_single_color_fill_t single_color_white_right = {
    .color = {.r = 0xff, .g=0xff, .b=0xff},
    .sides = SIDE_RIGHT
};

static const anim_conf_single_color_fill_t single_color_red_both = {
    .color = {.r = 0xff, .g=0, .b=0},
    .sides = SIDE_BOTH
};

static const anim_conf_single_color_fill_t single_color_red_left = {
    .color = {.r = 0xff, .g=0, .b=0},
    .sides = SIDE_LEFT
};

static const anim_conf_single_color_fill_t single_color_red_right = {
    .color = {.r = 0xff, .g=0, .b=0},
    .sides = SIDE_RIGHT
};


static const animation_t animations[] = {
{
    .render = single_color_chase_render,
    .user_data = &single_color_chase_red1
},
{
    .render = single_color_chase_render,
    .user_data = &single_color_chase_red2
},
{
    .render = multi_color_chase_render,
    .user_data = &multi_color_chase1
},
{
    .render = multi_color_chase_render,
    .user_data = &multi_color_chase2
},
{
    .render = multi_color_chase_render,
    .user_data = &multi_color_chase3
},
{
    .render = multi_color_chase_render,
    .user_data = &multi_color_chase4
},
{
    .render = multi_color_chase_render,
    .user_data = &multi_color_chase_rainbow
},
{
    .render = single_color_prbs_render,
    .reset = single_color_prbs_reset,
    .user_data = &single_color_prbs1
},
{
    .render = rgb_prbs_render,
    .reset = rgb_prbs_reset,
    .user_data = &rgb_prbs1
},
{
    .render = single_color_fill_render,
    .user_data = &single_color_white_both,
},
{
    .render = single_color_fill_render,
    .user_data = &single_color_white_left,
},
{
    .render = single_color_fill_render,
    .user_data = &single_color_white_right,
},
{
    .render = single_color_fill_render,
    .user_data = &single_color_red_both,
},
{
    .render = single_color_fill_render,
    .user_data = &single_color_red_left,
},
{
    .render = single_color_fill_render,
    .user_data = &single_color_red_right,
},
}; 

static union {
    anim_state_single_color_chase_t a;
    anim_state_multi_color_chase_t b;
    anim_state_prbs_t c;
} all_anim_state;

void animation_set_pattern(uint8_t pat)
{
    if(pat >= ARRAY_SIZE(animations))
        return;
    if(animation_pattern_current != pat) {
        memset(&all_anim_state, 0, sizeof(all_anim_state));
        const animation_t *a = &animations[pat];
        if(a->reset)
            a->reset(&all_anim_state, a->user_data);
        animation_pattern_current = pat;
    }
}


void animation_render(led_t *buf, uint8_t step)
{
    if(animation_pattern_current >= ARRAY_SIZE(animations))
        return;
    const animation_t *a = &animations[animation_pattern_current];
    a->render(buf, step, &all_anim_state, a->user_data);
}
