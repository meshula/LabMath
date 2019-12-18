
/* ============================================================================
 *
 *                                  3D DRAW
 *
 * =========================================================================== */
#include <GL/gl.h>
#include <GL/glu.h>

#define glLine(a,b) glLinef((a)[0],(a)[1],(a)[2],(b)[0],(b)[1],(b)[2])
#define glBox(c,w,h,d) glBoxf((c)[0],(c)[1],(c)[2],w,h,d)
#define glTriangle(a,b,c) glTrianglef((a)[0],(a)[1],(a)[2],(b)[0],(b)[1],(b)[2],(c)[0],(c)[1],(c)[2])
#define glSphere(c,r) glSpheref((c)[0],(c)[1],(c)[2],r)
#define glPlane(p,n,scale) glPlanef(p[0],p[1],p[2],n[0],n[1],n[2],scale)
#define glArrow(f,t,size) glArrowf((f)[0],(f)[1],(f)[2],(t)[0],(t)[1],(t)[2],size)
#define glBox(c,w,h,d) glBoxf((c)[0],(c)[1],(c)[2],w,h,d)
#define glCapsule(a,b,r) glCapsulef((a)[0],(a)[1],(a)[2], (b)[0],(b)[1],(b)[2], (r))
#define glPyramid(a,b,s) glPyramidf((a)[0],(a)[1],(a)[2], (b)[0],(b)[1],(b)[2], (s))

#define glError() glerror_(__FILE__, __LINE__)
static void
glerror_(const char *file, int line)
{
    const GLenum code = glGetError();
    if (code == GL_INVALID_ENUM)
        fprintf(stdout, "[GL] Error: (%s:%d) invalid value!\n", file, line);
    else if (code == GL_INVALID_OPERATION)
        fprintf(stdout, "[GL] Error: (%s:%d) invalid operation!\n", file, line);
    else if (code == GL_INVALID_FRAMEBUFFER_OPERATION)
        fprintf(stdout, "[GL] Error: (%s:%d) invalid frame op!\n", file, line);
    else if (code == GL_OUT_OF_MEMORY)
        fprintf(stdout, "[GL] Error: (%s:%d) out of memory!\n", file, line);
    else if (code == GL_STACK_UNDERFLOW)
        fprintf(stdout, "[GL] Error: (%s:%d) stack underflow!\n", file, line);
    else if (code == GL_STACK_OVERFLOW)
        fprintf(stdout, "[GL] Error: (%s:%d) stack overflow!\n", file, line);
}
static void
f3ortho(float *left, float *up, const float *v)
{
    #define inv_sqrt(x) (1.0f/sqrtf(x));
    float lenSqr, invLen;
    if (fabs(v[2]) > 0.7f) {
        lenSqr  = v[1]*v[1]+v[2]*v[2];
        invLen  = inv_sqrt(lenSqr);

        up[0] = 0.0f;
        up[1] =  v[2]*invLen;
        up[2] = -v[1]*invLen;

        left[0] = lenSqr*invLen;
        left[1] = -v[0]*up[2];
        left[2] =  v[0]*up[1];
    } else {
        lenSqr = v[0]*v[0] + v[1]*v[1];
        invLen = inv_sqrt(lenSqr);

        left[0] = -v[1] * invLen;
        left[1] =  v[0] * invLen;
        left[2] = 0.0f;

        up[0] = -v[2] * left[1];
        up[1] =  v[2] * left[0];
        up[2] = lenSqr * invLen;
    }
}
static void
glLinef(float ax, float ay, float az, float bx, float by, float bz)
{
    glBegin(GL_LINES);
    glVertex3f(ax, ay, az);
    glVertex3f(bx, by, bz);
    glEnd();
}
static void
glPlanef(float px, float py, float pz,
    float nx, float ny, float nz, float scale)
{
    float n[3], p[3];
    float v1[3], v2[3], v3[3], v4[3];
    float tangent[3], bitangent[3];

    f3set(n,nx,ny,nz);
    f3set(p,px,py,pz);
    f3ortho(tangent, bitangent, n);
    #define DD_PLANE_V(v, op1, op2) \
        v[0] = (p[0] op1 (tangent[0]*scale) op2 (bitangent[0]*scale)); \
        v[1] = (p[1] op1 (tangent[1]*scale) op2 (bitangent[1]*scale)); \
        v[2] = (p[2] op1 (tangent[2]*scale) op2 (bitangent[2]*scale))

    DD_PLANE_V(v1,-,-);
    DD_PLANE_V(v2,+,-);
    DD_PLANE_V(v3,+,+);
    DD_PLANE_V(v4,-,+);
    #undef DD_PLANE_V

    glLine(v1,v2);
    glLine(v2,v3);
    glLine(v3,v4);
    glLine(v4,v1);
}
static void
glTrianglef(float ax, float ay, float az,
    float bx, float by, float bz,
    float cx, float cy, float cz)
{
    glLinef(ax,ay,az, bx,by,bz);
    glLinef(bx,by,bz, cx,cy,cz);
    glLinef(cx,cy,cz, ax,ay,az);
}
static void
glArrowf(float fx, float fy, float fz, float tx, float ty, float tz, const float size)
{
    int i = 0;
    float degrees = 0.0f;
    static const float arrow_step = 30.0f;
    static const float arrow_sin[45] = {
        0.0f, 0.5f, 0.866025f, 1.0f, 0.866025f, 0.5f, -0.0f, -0.5f, -0.866025f,
        -1.0f, -0.866025f, -0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };
    static const float arrow_cos[45] = {
        1.0f, 0.866025f, 0.5f, -0.0f, -0.5f, -0.866026f, -1.0f, -0.866025f, -0.5f, 0.0f,
        0.5f, 0.866026f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    }; float from[3], to[3];
    float up[3], right[3], forward[3];

    f3set(from,fx,fy,fz);
    f3set(to,tx,ty,tz);
    f3sub(forward, to, from);
    f3norm(forward);
    f3ortho(right, up, forward);
    f3mul(forward, forward, size);

    glLine(from, to);
    for (i = 0; degrees < 360.0f; degrees += arrow_step, ++i) {
        float scale;
        float v1[3], v2[3], temp[3];

        scale = 0.5f * size * arrow_cos[i];
        f3mul(temp, right, scale);
        f3sub(v1, to, forward);
        f3add(v1, v1, temp);

        scale = 0.5f * size * arrow_sin[i];
        f3mul(temp, up, scale);
        f3add(v1, v1, temp);

        scale = 0.5f * size * arrow_cos[i + 1];
        f3mul(temp, right, scale);
        f3sub(v2, to, forward);
        f3add(v2, v2, temp);

        scale = 0.5f * size * arrow_sin[i + 1];
        f3mul(temp, up, scale);
        f3add(v2, v2, temp);

        glLine(v1, to);
        glLine(v1, v2);
    }
}
static void
glSpheref(float cx, float cy, float cz, const float radius)
{
    #define STEP_SIZE 15
    float last[3], tmp[3], radius_vec[3];
    float cache[(360 / STEP_SIZE)*3];
    int j = 0, i = 0, n = 0;

    f3set(radius_vec,0,0,radius);
    f3set(&cache[0], cx,cy,cz+radius_vec[2]);
    for (n = 1; n < (cntof(cache)/3); ++n)
        f3cpy(&cache[n*3], &cache[0]);

    /* first circle iteration */
    for (i = STEP_SIZE; i <= 360; i += STEP_SIZE) {
        const float s = sinf(i*TO_RAD);
        const float c = cosf(i*TO_RAD);

        last[0] = cx;
        last[1] = cy + radius * s;
        last[2] = cz + radius * c;

        /* second circle iteration */
        for (n = 0, j = STEP_SIZE; j <= 360; j += STEP_SIZE, ++n) {
            tmp[0] = cx + sinf(TO_RAD*j)*radius*s;
            tmp[1] = cy + cosf(TO_RAD*j)*radius*s;
            tmp[2] = last[2];

            glLine(last, tmp);
            glLine(last, &cache[n*3]);

            f3cpy(&cache[n*3], last);
            f3cpy(last, tmp);
        }
    }
    #undef STEP_SIZE
}
static void
glCapsulef(float fx, float fy, float fz, float tx, float ty, float tz, float r)
{
    int i = 0;
    static const int step_size = 20;
    float up[3], right[3], forward[3];
    float from[3], to[3];
    float lastf[3], lastt[3];

    /* calculate axis */
    f3set(from,fx,fy,fz);
    f3set(to,tx,ty,tz);
    f3sub(forward, to, from);
    f3norm(forward);
    f3ortho(right, up, forward);

    /* calculate first two cone verts (buttom + top) */
    f3mul(lastf, up,r);
    f3add(lastt, to,lastf);
    f3add(lastf, from,lastf);

    /* step along circle outline and draw lines */
    for (i = step_size; i <= 360; i += step_size) {
        /* calculate current rotation */
        float ax[3], ay[3], pf[3], pt[3], tmp[3];
        f3mul(ax, right, sinf(i*TO_RAD));
        f3mul(ay, up, cosf(i*TO_RAD));

        /* calculate current vertices on cone */
        f3add(tmp, ax, ay);
        f3mul(pf, tmp, r);
        f3mul(pt, tmp, r);

        f3add(pf, pf, from);
        f3add(pt, pt, to);

        /* draw cone vertices */
        glLine(lastf, pf);
        glLine(lastt, pt);
        glLine(pf, pt);

        f3cpy(lastf, pf);
        f3cpy(lastt, pt);

        /* calculate first top sphere vert */
        float prevt[3], prevf[3];
        f3mul(prevt, tmp, r);
        f3add(prevf, prevt, from);
        f3add(prevt, prevt, to);

        /* sphere (two half spheres )*/
        for (int j = 1; j < 180/step_size; j++) {
            /* angles */
            float ta = j*step_size;
            float fa = 360-(j*step_size);
            float t[3];

            /* top half-sphere */
            f3mul(ax, forward, sinf(ta*TO_RAD));
            f3mul(ay, tmp, cosf(ta*TO_RAD));

            f3add(t, ax, ay);
            f3mul(pf, t, r);
            f3add(pf, pf, to);
            glLine(pf, prevt);
            f3cpy(prevt, pf);

            /* bottom half-sphere */
            f3mul(ax, forward, sinf(fa*TO_RAD));
            f3mul(ay, tmp, cosf(fa*TO_RAD));

            f3add(t, ax, ay);
            f3mul(pf, t, r);
            f3add(pf, pf, from);
            glLine(pf, prevf);
            f3cpy(prevf, pf);
        }
    }
}
static void
pyramid(float *dst, float fx, float fy, float fz, float tx, float ty, float tz, float size)
{
    float from[3];
    float *a = dst + 0;
    float *b = dst + 3;
    float *c = dst + 6;
    float *d = dst + 9;
    float *r = dst + 12;

    /* calculate axis */
    float up[3], right[3], forward[3];
    f3set(from,fx,fy,fz);
    f3set(r,tx,ty,tz);
    f3sub(forward, r, from);
    f3norm(forward);
    f3ortho(right, up, forward);

    /* calculate extend */
    float xext[3], yext[3];
    float nxext[3], nyext[3];
    f3mul(xext, right, size);
    f3mul(yext, up, size);
    f3mul(nxext, right, -size);
    f3mul(nyext, up, -size);

    /* calculate base vertexes */
    f3add(a, from, xext);
    f3add(a, a, yext);

    f3add(b, from, xext);
    f3add(b, b, nyext);

    f3add(c, from, nxext);
    f3add(c, c, nyext);

    f3add(d, from, nxext);
    f3add(d, d, yext);
}
static void
glPyramidf(float fx, float fy, float fz,
    float tx, float ty, float tz, float size)
{
    float pyr[16];
    pyramid(pyr, fx, fy, fz, tx, ty, tz, size);
    float *a = pyr + 0;
    float *b = pyr + 3;
    float *c = pyr + 6;
    float *d = pyr + 9;
    float *r = pyr + 12;

    /* draw vertexes */
    glLine(a, b);
    glLine(b, c);
    glLine(c, d);
    glLine(d, a);

    /* draw root */
    glLine(a, r);
    glLine(b, r);
    glLine(c, r);
    glLine(d, r);
}
static void
diamond(float *dst, float fx, float fy, float fz,
    float tx, float ty, float tz, float size)
{
    float d[3], f[3], t[3], from[3];
    f3set(t, tx,ty,tz);
    f3set(f, fx,fy,fz);
    f3sub(d, t, f);
    f3mul(d, d, 0.5f);
    f3add(from, f, d);

    pyramid(dst, from[0], from[1], from[2], tx, ty, tz, size);
    f3cpy(dst + 15, f);
}
static void
glDiamondf(float fx, float fy, float fz,
    float tx, float ty, float tz, float size)
{
    float dmd[18];
    diamond(dmd, fx, fy, fz, tx, ty, tz, size);

    float *a = dmd + 0;
    float *b = dmd + 3;
    float *c = dmd + 6;
    float *d = dmd + 9;
    float *t = dmd + 12;
    float *f = dmd + 15;

    /* draw vertexes */
    glLine(a, b);
    glLine(b, c);
    glLine(c, d);
    glLine(d, a);

    /* draw roof */
    glLine(a, t);
    glLine(b, t);
    glLine(c, t);
    glLine(d, t);

    /* draw floor */
    glLine(a, f);
    glLine(b, f);
    glLine(c, f);
    glLine(d, f);
}
static void
glGrid(const float mins, const float maxs, const float y, const float step)
{
    float i;
    float from[3], to[3];
    for (i = mins; i <= maxs; i += step) {
        /* Horizontal line (along the X) */
        f3set(from, mins,y,i);
        f3set(to, maxs,y,i);
        glLine(from,to);
        /* Vertical line (along the Z) */
        f3set(from, i,y,mins);
        f3set(to, i,y,maxs);
        glLine(from,to);
    }
}
static void
glBounds(const float *points)
{
    int i = 0;
    for (i = 0; i < 4; ++i) {
        glLine(&points[i*3], &points[((i + 1) & 3)*3]);
        glLine(&points[(4 + i)*3], &points[(4 + ((i + 1) & 3))*3]);
        glLine(&points[i*3], &points[(4 + i)*3]);
    }
}
static void
glBoxf(float cx, float cy, float cz, float w, float h, float d)
{
    float pnts[8*3];
    w  = w*0.5f;
    h  = h*0.5f;
    d  = d*0.5f;

    #define DD_BOX_V(v, op1, op2, op3)\
        (v)[0] = cx op1 w; (v)[1] = cy op2 h; (v)[2] = cz op3 d
    DD_BOX_V(&pnts[0*3], -, +, +);
    DD_BOX_V(&pnts[1*3], -, +, -);
    DD_BOX_V(&pnts[2*3], +, +, -);
    DD_BOX_V(&pnts[3*3], +, +, +);
    DD_BOX_V(&pnts[4*3], -, -, +);
    DD_BOX_V(&pnts[5*3], -, -, -);
    DD_BOX_V(&pnts[6*3], +, -, -);
    DD_BOX_V(&pnts[7*3], +, -, +);
    #undef DD_BOX_V
    glBounds(pnts);
}
static void
glAabb(const float *bounds)
{
    int i = 0;
    float bb[2*3], pnts[8*3];
    f3cpy(&bb[0], bounds);
    f3cpy(&bb[3], bounds+3);
    for (i = 0; i < cntof(pnts)/3; ++i) {
        pnts[i*3+0] = bb[(((i ^ (i >> 1)) & 1)*3+0)];
        pnts[i*3+1] = bb[(((i >> 1) & 1)*3)+1];
        pnts[i*3+2] = bb[(((i >> 2) & 1)*3)+2];
    } glBounds(pnts);
}

/* ============================================================================
 *
 *                                  SYSTEM
 *
 * =========================================================================== */
#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>

typedef struct sys_int2 {int x,y;} sys_int2;
enum {
    SYS_FALSE = 0,
    SYS_TRUE = 1,
    SYS_MAX_KEYS = 256
};
struct sys_button {
    unsigned int down:1;
    unsigned int pressed:1;
    unsigned int released:1;
};
enum sys_mouse_mode {
    SYSTEM_MOUSE_ABSOLUTE,
    SYSTEM_MOUSE_RELATIVE
};
struct sys_mouse {
    enum sys_mouse_mode mode;
    unsigned visible:1;
    sys_int2 pos;
    sys_int2 pos_last;
    sys_int2 pos_delta;
    float scroll_delta;
    struct sys_button left_button;
    struct sys_button right_button;
};
struct sys_time {
    float refresh_rate;
    float last;
    float delta;
    unsigned int start;
};
enum sys_window_mode {
    SYSTEM_WM_WINDOWED,
    SYSTEM_WM_FULLSCREEN,
    SYSTEM_WM_MAX,
};
struct sys_window {
    const char *title;
    SDL_Window *handle;
    SDL_GLContext glContext;
    enum sys_window_mode mode;
    sys_int2 pos;
    sys_int2 size;
    unsigned resized:1;
    unsigned moved:1;
};
struct sys {
    unsigned quit:1;
    unsigned initialized:1;
    struct sys_time time;
    struct sys_window win;
    struct sys_button key[SYS_MAX_KEYS];
    struct sys_mouse mouse;
};
static void
sys_init(struct sys *s)
{
    int monitor_refresh_rate;
    int win_x, win_y, win_w, win_h;
    const char *title;

    SDL_Init(SDL_INIT_VIDEO);
    title = s->win.title ? s->win.title: "SDL";
    win_x = s->win.pos.x ? s->win.pos.x: SDL_WINDOWPOS_CENTERED;
    win_y = s->win.pos.y ? s->win.pos.y: SDL_WINDOWPOS_CENTERED;
    win_w = s->win.size.x ? s->win.size.x: 800;
    win_h = s->win.size.y ? s->win.size.y: 600;
    s->win.handle = SDL_CreateWindow(title, win_x, win_y, win_w, win_h, SDL_WINDOW_OPENGL|SDL_WINDOW_SHOWN);

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);
    s->win.glContext = SDL_GL_CreateContext(s->win.handle);
    s->mouse.visible = SYS_TRUE;

    monitor_refresh_rate = 60;
    s->time.refresh_rate = 1.0f/(float)monitor_refresh_rate;
    s->initialized = SYS_TRUE;
}
static void
sys_shutdown(struct sys *s)
{
    SDL_GL_DeleteContext(s->win.glContext);
    SDL_DestroyWindow(s->win.handle);
    SDL_Quit();
}
static void
sys_update_button(struct sys_button *b, unsigned down)
{
    int was_down = b->down;
    b->down = down;
    b->pressed = !was_down && down;
    b->released = was_down && !down;
}
static void
sys_pull(struct sys *s)
{
    int i = 0;
    SDL_PumpEvents();
    s->time.start = SDL_GetTicks();
    s->mouse.left_button.pressed = 0;
    s->mouse.left_button.released = 0;
    s->mouse.right_button.pressed = 0;
    s->mouse.right_button.released = 0;
    s->mouse.scroll_delta = 0;

    /* window-mode */
    if (s->win.mode == SYSTEM_WM_WINDOWED)
        SDL_SetWindowFullscreen(s->win.handle, 0);
    else SDL_SetWindowFullscreen(s->win.handle, SDL_WINDOW_FULLSCREEN_DESKTOP);

    /* poll window */
    s->win.moved = 0;
    s->win.resized = 0;
    SDL_GetWindowSize(s->win.handle, &s->win.size.x, &s->win.size.y);
    SDL_GetWindowPosition(s->win.handle, &s->win.pos.x, &s->win.pos.y);

    /* poll keyboard */
    {const unsigned char* keys = SDL_GetKeyboardState(0);
    for (i = 0; i < SYS_MAX_KEYS; ++i) {
        SDL_Keycode sym = SDL_GetKeyFromScancode((SDL_Scancode)i);
        if (sym < SYS_MAX_KEYS)
            sys_update_button(s->key + sym, keys[i]);
    }}
    /* poll mouse */
    s->mouse.pos_delta.x = 0;
    s->mouse.pos_delta.y = 0;
    SDL_SetRelativeMouseMode(s->mouse.mode == SYSTEM_MOUSE_RELATIVE);
    if (s->mouse.mode == SYSTEM_MOUSE_ABSOLUTE) {
        s->mouse.pos_last.x = s->mouse.pos.x;
        s->mouse.pos_last.y = s->mouse.pos.y;
        SDL_GetMouseState(&s->mouse.pos.x, &s->mouse.pos.y);
        s->mouse.pos_delta.x = s->mouse.pos.x - s->mouse.pos_last.x;
        s->mouse.pos_delta.y = s->mouse.pos.y - s->mouse.pos_last.y;
    } else {
        s->mouse.pos_last.x = s->win.size.x/2;
        s->mouse.pos_last.y = s->win.size.y/2;
        s->mouse.pos.x = s->win.size.x/2;
        s->mouse.pos.y = s->win.size.y/2;
    }
    /* handle events */
    {SDL_Event evt;
    while (SDL_PollEvent(&evt)) {
        if (evt.type == SDL_QUIT)
            s->quit = SYS_TRUE;
        else if (evt.type == SDL_MOUSEBUTTONDOWN ||  evt.type == SDL_MOUSEBUTTONUP) {
            unsigned down = evt.type == SDL_MOUSEBUTTONDOWN;
            if (evt.button.button == SDL_BUTTON_LEFT)
                sys_update_button(&s->mouse.left_button, down);
            else if (evt.button.button == SDL_BUTTON_RIGHT)
                sys_update_button(&s->mouse.right_button, down);
        } else if (evt.type == SDL_WINDOWEVENT) {
            if (evt.window.event == SDL_WINDOWEVENT_MOVED)
                s->win.moved = SYS_TRUE;
            else if (evt.window.event == SDL_WINDOWEVENT_RESIZED)
                s->win.resized = SYS_TRUE;
        } else if (evt.type == SDL_MOUSEMOTION) {
            s->mouse.pos_delta.x += evt.motion.xrel;
            s->mouse.pos_delta.y += evt.motion.yrel;
        } else if (evt.type == SDL_MOUSEWHEEL) {
            s->mouse.scroll_delta = -evt.wheel.y;
        }
    }}
}
static void
sys_push(struct sys *s)
{
    SDL_GL_SwapWindow(s->win.handle);
    s->time.delta = (SDL_GetTicks() - s->time.start) / 1000.0f;
    if (s->time.delta < s->time.refresh_rate)
        SDL_Delay((Uint32)((s->time.refresh_rate - s->time.delta) * 1000.0f));
}

/* ============================================================================
 *
 *                                  APP
 *
 * =========================================================================== */
int main(void)
{
    /* Platform */
    struct sys sys;
    memset(&sys, 0, sizeof(sys));
    sys.win.title = "Demo";
    sys.win.size.x = 1200;
    sys.win.size.y = 800;
    sys_init(&sys);

    /* Camera */
    struct cam cam;
    cam_init(&cam);
    cam.pos[2] = 5;
    cam_build(&cam);

    /* Main */
    while (!sys.quit)
    {
        sys_pull(&sys);

        /* Camera control */
        #define CAMERA_SPEED (10.0f)
        {const float frame_movement = CAMERA_SPEED * sys.time.delta;
        const float forward = (sys.key['w'].down ? -1 : 0.0f) + (sys.key['s'].down ? 1 : 0.0f);
        const float right = (sys.key['a'].down ? -1 : 0.0f) + (sys.key['d'].down ? 1 : 0.0f);
        if (sys.mouse.right_button.down) {
            sys.mouse.mode = SYSTEM_MOUSE_RELATIVE;
            cam.ear[0] = (float)sys.mouse.pos_delta.y * TO_RAD;
            cam.ear[1] = (float)sys.mouse.pos_delta.x * TO_RAD;
        } else sys.mouse.mode = SYSTEM_MOUSE_ABSOLUTE;
        if (sys.key['e'].down)
            cam.ear[2] = 1.0f * TO_RAD;
        if (sys.key['r'].down)
            cam.ear[2] = -1.0f * TO_RAD;
        cam.off[2] += sys.mouse.scroll_delta;
        cam.aspect_ratio = (float)sys.win.size.x/(float)sys.win.size.y;
        cam_move(&cam, right * frame_movement, 0, forward * frame_movement);
        cam_build(&cam);}

        /* Keybindings */
        if (sys.key['f'].pressed)
            sys.win.mode = !sys.win.mode;
        if (sys.key['b'].pressed)
            printf("break!\n");

        /* Rendering */
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        glClearColor(0.15f, 0.15f, 0.15f, 1);
        glViewport(0, 0, sys.win.size.x, sys.win.size.y);
        {
            /* 3D */
            glEnable(GL_DEPTH_TEST);
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glLoadMatrixf((float*)cam.proj);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            glLoadMatrixf((float*)cam.view);

            /* grid */
            glColor3f(0.2f,0.2f,0.2f);
            glGrid(-50.0f, 50.0f, -1.0f, 1.7f);

            /* basis-axis */
            glColor3f(1,0,0);
            glArrowf(0,0,0, 1,0,0, 0.1f);
            glColor3f(0,1,0);
            glArrowf(0,0,0, 0,1,0, 0.1f);
            glColor3f(0,0,1);
            glArrowf(0,0,0, 0,0,1, 0.1f);

            {
                /* Triangle-Ray Intersection*/
                static float dx = 0, dy = 0;
                float ro[3], rd[3];
                int suc;

                v3 tri[3];
                tri[0] = v3mk(-9,1,28);
                tri[1] = v3mk(-10,0,28);
                tri[2] = v3mk(-11,1,28);

                /* ray */
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;
                f3set(ro, -10,-1,20);
                f3set(rd, -10+0.4f*sinf(dx), 2.0f*cosf(dy), 29.81023f);
                f3sub(rd, rd, ro);
                f3norm(rd);

                struct raycast hit;
                struct ray r = rayfv(ro, rd);
                if ((suc = ray_test_triangle(&hit, r, tri))) {
                    /* point of intersection */
                    glColor3f(1,0,0);
                    glBox(&hit.p.x, 0.10f, 0.10f, 0.10f);

                    /* intersection normal */
                    glColor3f(0,0,1);
                    v3 v = v3add(hit.p, hit.n);
                    glArrow(&hit.p.x, &v.x, 0.05f);
                }

                /* line */
                glColor3f(1,0,0);
                f3mul(rd,rd,10);
                f3add(rd,ro,rd);
                glLine(ro, rd);

                /* triangle */
                if (suc) glColor3f(1,0,0);
                else glColor3f(1,1,1);
                glTriangle(&tri[0].x,&tri[1].x,&tri[2].x);
            }
            {
                /* Plane-Ray Intersection*/
                static float d = 0;
                struct ray ray;
                struct plane plane;
                struct raycast hit;
                float ro[3], rd[3];
                int suc = 0;
                m3 rot;

                /* ray */
                d += sys.time.delta * 2.0f;
                f3set(ro, 0,-1,20);
                f3set(rd, 0.1f, 0.5f, 9.81023f);
                f3sub(rd, rd, ro);
                f3norm(rd);

                /* rotation */
                m3cpy(rot, m3id);
                m3rot((float*)rot, d, 0,1,0);
                m3mulv(rd,(float*)rot,rd);

                /* intersection */
                ray = rayfv(ro, rd);
                plane = planefv(0,0,28, 0,0,1);
                if ((suc = ray_test_plane(&hit, ray, plane))) {
                    /* point of intersection */
                    glColor3f(1,0,0);
                    glBox(&hit.p.x, 0.10f, 0.10f, 0.10f);

                    /* intersection normal */
                    glColor3f(0,0,1);
                    v3 v = v3add(hit.p, hit.n);
                    glArrow(&hit.p.x, &v.x, 0.05f);
                    glColor3f(1,0,0);
                }
                /* line */
                glColor3f(1,0,0);
                f3mul(rd,rd,9);
                f3add(rd,ro,rd);
                glLine(ro, rd);

                /* plane */
                if (suc) glColor3f(1,0,0);
                else glColor3f(1,1,1);
                glPlanef(0,0,28, 0,0,1, 3.0f);
            }
            {
                /* Sphere-Ray Intersection*/
                static float dx = 0, dy = 0;
                float ro[3], rd[3];
                struct sphere s;
                struct raycast hit;
                struct ray r;
                int suc;

                /* ray */
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                f3set(ro, 0,-1,0);
                f3set(rd, 0.4f*sinf(dx), 2.0f*cosf(dy), 9.81023f);
                f3sub(rd, rd, ro);
                f3norm(rd);

                r = rayfv(ro, rd);
                s = sphere(v3mk(0,0,8), 1);
                if ((suc = ray_test_sphere(&hit, r, s))) {
                    /* points of intersection */
                    float in[3];
                    f3mul(in,rd,hit.t0);
                    f3add(in,ro,in);

                    glColor3f(1,0,0);
                    glBox(in, 0.05f, 0.05f, 0.05f);

                    f3mul(in,rd,hit.t1);
                    f3add(in,ro,in);

                    glColor3f(1,0,0);
                    glBox(in, 0.05f, 0.05f, 0.05f);

                    /* intersection normal */
                    glColor3f(0,0,1);
                    v3 v = v3add(hit.p, hit.n);
                    glArrow(&hit.p.x, &v.x, 0.05f);
                    glColor3f(1,0,0);
                }
                /* line */
                glColor3f(1,0,0);
                f3mul(rd,rd,10);
                f3add(rd,ro,rd);
                glLine(ro, rd);

                /* sphere */
                if (suc) glColor3f(1,0,0);
                else glColor3f(1,1,1);
                glSpheref(0,0,8, 1);
            }
            {
                /* AABB-Ray Intersection*/
                static float dx = 0, dy = 0;
                struct ray ray;
                struct aabb bounds;
                struct raycast hit;
                int suc = 0;
                float ro[3], rd[3];

                /* ray */
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                f3set(ro, 10,-1,0);
                f3set(rd, 10+0.4f*sinf(dx), 2.0f*cosf(dy), 9.81023f);
                f3sub(rd, rd, ro);
                f3norm(rd);

                ray = rayfv(ro, rd);
                bounds = aabb(v3mk(10-0.5f,-0.5f,7.5f), v3mk(10.5f,0.5f,8.5f));
                if ((suc = ray_test_aabb(&hit, ray, bounds))) {
                    /* points of intersection */
                    float in[3];
                    f3mul(in,rd,hit.t0);
                    f3add(in,ro,in);

                    glColor3f(1,0,0);
                    glBox(in, 0.05f, 0.05f, 0.05f);

                    f3mul(in,rd,hit.t1);
                    f3add(in,ro,in);

                    glColor3f(1,0,0);
                    glBox(in, 0.05f, 0.05f, 0.05f);

                    /* intersection normal */
                    glColor3f(0,0,1);
                    v3 v = v3add(hit.p, hit.n);
                    glArrow(&hit.p.x, &v.x, 0.05f);
                    glColor3f(1,0,0);
                } else glColor3f(1,1,1);
                glBoxf(10,0,8, 1,1,1);

                /* line */
                glColor3f(1,0,0);
                f3mul(rd,rd,10);
                f3add(rd,ro,rd);
                glLine(ro, rd);
            }
            {
                /* Sphere-Sphere intersection*/
                int suc = 0;
                struct manifold m;
                static float dx = 0, dy = 0;
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                struct sphere a = sphere(v3mk(-10,0,8), 1);
                struct sphere b = sphere(v3mk(-10+0.6f*sinf(dx), 3.0f*cosf(dy),8), 1);
                if ((suc = sphere_test_sphere_manifold(&m, a, b))) {
                    float v[3];
                    glColor3f(0,0,1);
                    glBox(m.contact_point, 0.05f, 0.05f, 0.05f);
                    f3add(v, m.contact_point, m.normal);
                    glArrow(m.contact_point, v, 0.05f);
                    glColor3f(1,0,0);
                } else glColor3f(1,1,1);

                glSphere(&a.p.x, 1);
                glSphere(&b.p.x, 1);
            }
            {
                /* AABB-AABB intersection*/
                int suc = 0;
                struct manifold m;
                static float dx = 0, dy = 0;
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                const float x = 10+0.6f*sinf(dx);
                const float y = 3.0f*cosf(dy);
                const float z = 20.0f;

                struct aabb a = aabb(v3mk(10-0.5f,-0.5f,20-0.5f), v3mk(10+0.5f,0.5f,20.5f));
                struct aabb b = aabbf(x-0.5f,y-0.5f,z-0.5f, x+0.5f,y+0.5f,z+0.5f);
                if ((suc = aabb_test_aabb_manifold(&m, a, b))) {
                    float v[3];
                    glColor3f(0,0,1);
                    glBox(m.contact_point, 0.05f, 0.05f, 0.05f);
                    f3add(v, m.contact_point, m.normal);
                    glArrow(m.contact_point, v, 0.05f);
                    glColor3f(1,0,0);
                } else glColor3f(1,1,1);

                glBoxf(10,0,20, 1,1,1);
                glBoxf(x,y,z, 1,1,1);
            }
            {
                /* Capsule-Capsule intersection*/
                int suc = 0;
                struct manifold m;
                static float dx = 0, dy = 0;
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                const float x = 20+0.4f*sinf(dx);
                const float y = 3.0f*cosf(dy);
                const float z = 28.5f;

                struct capsule a = capsulef(20.0f,-1.0f,28.0f, 20.0f,1.0f,28.0f, 0.2f);
                struct capsule b = capsulef(x,y-1.0f,z, x,y+1.0f,z-1.0f, 0.2f);
                if ((suc = capsule_test_capsule_manifold(&m, a, b))) {
                    float v[3];
                    glColor3f(0,0,1);
                    glBox(m.contact_point, 0.05f, 0.05f, 0.05f);
                    f3add(v, m.contact_point, m.normal);
                    glArrow(m.contact_point, v, 0.05f);
                    glColor3f(1,0,0);
                } else glColor3f(1,1,1);
                glCapsulef(x,y-1.0f,z, x,y+1.0f,z-1.0f, 0.2f);
                glCapsulef(20.0f,-1.0f,28.0f, 20.0f,1.0f,28.0f, 0.2f);
            }
            {
                /* AABB-Sphere intersection*/
                int suc = 0;
                struct manifold m;
                static float dx = 0, dy = 0;
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                struct aabb a = aabb(v3mk(20-0.5f,-0.5f,7.5f), v3mk(20.5f,0.5f,8.5f));
                struct sphere s = sphere(v3mk(20+0.6f*sinf(dx), 3.0f*cosf(dy),8), 1);
                if ((suc = aabb_test_sphere_manifold(&m, a, s))) {
                    float v[3];
                    glColor3f(0,0,1);
                    glBox(m.contact_point, 0.05f, 0.05f, 0.05f);
                    f3add(v, m.contact_point, m.normal);
                    glArrow(m.contact_point, v, 0.05f);
                    glColor3f(1,0,0);
                } else glColor3f(1,1,1);

                glBoxf(20,0,8, 1,1,1);
                glSphere(&s.p.x, 1);
            }
            {
                /* Sphere-AABB intersection*/
                int suc = 0;
                struct manifold m;
                static float dx = 0, dy = 0;
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                const float x = 10+0.6f*sinf(dx);
                const float y = 3.0f*cosf(dy);
                const float z = -8.0f;

                struct sphere s = sphere(v3mk(10,0,-8), 1);
                struct aabb a = aabbf(x-0.5f,y-0.5f,z-0.5f, x+0.5f,y+0.5f,z+0.5f);
                if ((suc = sphere_test_aabb_manifold(&m, s, a))) {
                    float v[3];
                    glColor3f(0,0,1);
                    glBox(m.contact_point, 0.05f, 0.05f, 0.05f);
                    f3add(v, m.contact_point, m.normal);
                    glArrow(m.contact_point, v, 0.05f);
                    glColor3f(1,0,0);
                } else glColor3f(1,1,1);

                glBoxf(x,y,z, 1,1,1);
                glSphere(&s.p.x, 1);
            }
            {
                /* Capsule-Sphere intersection*/
                int suc = 0;
                struct manifold m;
                static float dx = 0, dy = 0;
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                struct capsule c = capsulef(-20.5f,-1.0f,7.5f, -20+0.5f,1.0f,8.5f, 0.2f);
                struct sphere b = sphere(v3mk(-20+0.6f*sinf(dx), 3.0f*cosf(dy),8), 1);
                if ((suc = capsule_test_sphere_manifold(&m, c, b))) {
                    float v[3];
                    glColor3f(0,0,1);
                    glBox(m.contact_point, 0.05f, 0.05f, 0.05f);
                    f3add(v, m.contact_point, m.normal);
                    glArrow(m.contact_point, v, 0.05f);
                    glColor3f(1,0,0);
                } else glColor3f(1,1,1);
                glSpheref(b.p.x, b.p.y, b.p.z, 1);
                glCapsulef(-20.5f,-1.0f,7.5f, -20+0.5f,1.0f,8.5f, 0.2f);
            }
            {
                /* Sphere-Capsule intersection*/
                int suc = 0;
                struct manifold m;
                static float dx = 0, dy = 0;
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                const float x = 20+0.4f*sinf(dx);
                const float y = 3.0f*cosf(dy);
                const float z = -8;

                struct sphere s = sphere(v3mk(20,0,-8), 1);
                struct capsule c = capsulef(x,y-1.0f,z, x,y+1.0f,z-1.0f, 0.2f);
                if ((suc = sphere_test_capsule_manifold(&m, s, c))) {
                    float v[3];
                    glColor3f(0,0,1);
                    glBox(m.contact_point, 0.05f, 0.05f, 0.05f);
                    f3add(v, m.contact_point, m.normal);
                    glArrow(m.contact_point, v, 0.05f);
                    glColor3f(1,0,0);
                } else glColor3f(1,1,1);

                glCapsulef(x,y-1.0f,z, x,y+1.0f,z-1.0f, 0.2f);
                glSphere(&s.p.x, 1);
            }
            {
                /* Capsule-AABB intersection*/
                int suc = 0;
                struct manifold m;
                static float dx = 0, dy = 0;
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                const float x = -20+0.6f*sinf(dx);
                const float y = 3.0f*cosf(dy);
                const float z = 28.0f;

                struct capsule c = capsulef(-20.5f,-1.0f,27.5f, -20+0.5f,1.0f,28.5f, 0.2f);
                struct aabb b = aabbf(x-0.5f,y-0.5f,z-0.5f, x+0.5f,y+0.5f,z+0.5f);
                if ((suc = capsule_test_aabb_manifold(&m, c, b))) {
                    float v[3];
                    glColor3f(0,0,1);
                    glBox(m.contact_point, 0.05f, 0.05f, 0.05f);
                    f3add(v, m.contact_point, m.normal);
                    glArrow(m.contact_point, v, 0.05f);
                    glColor3f(1,0,0);
                } else glColor3f(1,1,1);
                glBoxf(x,y,z, 1,1,1);
                glCapsulef(-20.5f,-1.0f,27.5f, -20+0.5f,1.0f,28.5f, 0.2f);
            }
            {
                /* AABB-Capsule intersection*/
                int suc = 0;
                struct manifold m;
                static float dx = 0, dy = 0;
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                const float x = 0.4f*sinf(dx);
                const float y = 3.0f*cosf(dy);
                const float z = -8;

                struct aabb a = aabb(v3mk(-0.5f,-0.5f,-8.5f), v3mk(0.5f,0.5f,-7.5f));
                struct capsule c = capsulef(x,y-1.0f,z, x,y+1.0f,z-1.0f, 0.2f);
                if ((suc = aabb_test_capsule_manifold(&m, a, c))) {
                    float v[3];
                    glColor3f(1,0,0);
                    glBox(m.contact_point, 0.05f, 0.05f, 0.05f);
                    f3add(v, m.contact_point, m.normal);
                    glArrow(m.contact_point, v, 0.05f);
                } else glColor3f(1,1,1);

                glCapsulef(x,y-1.0f,z, x,y+1.0f,z-1.0f, 0.2f);
                glBoxf(0,0,-8.0f, 1,1,1);
            }
            {
                /* Polyhedron(Pyramid)-Sphere (GJK) intersection*/
                static float dx = 0, dy = 0;
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                float pyr[15];
                struct gjk_result gjk;
                struct sphere s = sphere(v3mk(-10+0.6f*sinf(dx), 3.0f*cosf(dy),-8), 1);
                pyramid(pyr, -10.5f,-0.5f,-7.5f, -10.5f,1.0f,-7.5f, 1.0f);
                if (polyhedron_intersect_sphere(&gjk, pyr, 5, &s.p.x, s.r))
                    glColor3f(1,0,0);
                else glColor3f(1,1,1);

                glSphere(&s.p.x, 1);
                glPyramidf(-10.5f,-0.5f,-7.5f, -10.5f,1.0f,-7.5f, 1.0f);

                glBox(gjk.p0, 0.05f, 0.05f, 0.05f);
                glBox(gjk.p1, 0.05f, 0.05f, 0.05f);
                glLine(gjk.p0, gjk.p1);
            }
            {
                /* Polyhedron(Diamond)-Sphere (GJK) intersection*/
                static float dx = 0, dy = 0;
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                float dmd[18];
                struct gjk_result gjk;
                struct sphere s = sphere(v3mk(-20+0.6f*sinf(dx), 3.0f*cosf(dy),-8), 1);
                diamond(dmd, -20.5f,-0.5f,-7.5f, -20.5f,1.0f,-7.5f, 0.5f);
                if (polyhedron_intersect_sphere(&gjk, dmd, 6, &s.p.x, s.r))
                    glColor3f(1,0,0);
                else glColor3f(1,1,1);

                glSphere(&s.p.x, 1);
                glDiamondf(-20.5f,-0.5f,-7.5f, -20.5f,1.0f,-7.5f, 0.5f);

                glBox(gjk.p0, 0.05f, 0.05f, 0.05f);
                glBox(gjk.p1, 0.05f, 0.05f, 0.05f);
                glLine(gjk.p0, gjk.p1);
            }
            {
                /* Polyhedron(Pyramid)-Capsule (GJK) intersection*/
                static float dx = 0, dy = 0;
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                const float x = 0.4f*sinf(dx);
                const float y = 3.0f*cosf(dy);
                const float z = -15;

                float pyr[15];
                struct gjk_result gjk;
                struct capsule c = capsulef(x,y-1.0f,z, x,y+1.0f,z, 0.2f);
                pyramid(pyr, -0.5f,-0.5f,-15.5f, -0.5f,1.0f,-15.5f, 1.0f);

                if (polyhedron_intersect_capsule(&gjk, pyr, 5, &c.a.x, &c.b.x, c.r))
                    glColor3f(1,0,0);
                else glColor3f(1,1,1);

                glCapsule(&c.a.x, &c.b.x, c.r);
                glPyramidf(-0.5f,-0.5f,-15.5f, -0.5f,1.0f,-15.5f, 1.0f);

                glBox(gjk.p0, 0.05f, 0.05f, 0.05f);
                glBox(gjk.p1, 0.05f, 0.05f, 0.05f);
                glLine(gjk.p0, gjk.p1);
            }

            {
                /* Polyhedron(Diamond)-Capsule (GJK) intersection*/
                static float dx = 0, dy = 0;
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                const float x = -10 + 0.4f*sinf(dx);
                const float y = 3.0f*cosf(dy);
                const float z = -15;

                float dmd[18];
                struct gjk_result gjk;
                struct capsule c = capsulef(x,y-1.0f,z, x,y+1.0f,z, 0.2f);
                diamond(dmd, -10.5f,-0.5f,-15.5f, -10.5f,1.0f,-15.5f, 0.5f);

                if (polyhedron_intersect_capsule(&gjk, dmd, 6, &c.a.x, &c.b.x, c.r))
                    glColor3f(1,0,0);
                else glColor3f(1,1,1);

                glCapsule(&c.a.x, &c.b.x, c.r);
                glDiamondf(-10.5f,-0.5f,-15.5f, -10.5f,1.0f,-15.5f, 0.5f);

                glBox(gjk.p0, 0.05f, 0.05f, 0.05f);
                glBox(gjk.p1, 0.05f, 0.05f, 0.05f);
                glLine(gjk.p0, gjk.p1);
            }

            {
                /* Polyhedron(Diamond)-Polyhedron(Pyramid) (GJK) intersection*/
                static float dx = 0, dy = 0;
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                const float x = -20 + 0.4f*sinf(dx);
                const float y = 3.0f*cosf(dy);
                const float z = -15;

                float dmd[18], pyr[15];
                struct gjk_result gjk;

                pyramid(pyr, x,y-0.5f,z, x,y+1,z, 0.8f);
                diamond(dmd, -20.5f,-0.5f,-15.5f, -20.5f,1.0f,-15.5f, 0.5f);
                if (polyhedron_intersect_polyhedron(&gjk, dmd, 6, pyr, 5))
                    glColor3f(1,0,0);
                else glColor3f(1,1,1);

                glPyramidf(x,y-0.5f,z, x,y+1,z, 0.8f);
                glDiamondf(-20.5f,-0.5f,-15.5f, -20.5f,1.0f,-15.5f, 0.5f);

                glBox(gjk.p0, 0.05f, 0.05f, 0.05f);
                glBox(gjk.p1, 0.05f, 0.05f, 0.05f);
                glLine(gjk.p0, gjk.p1);
            }
            {
                /* Polyhedron(Pyramid)-Polyhedron(Diamond) (GJK) intersection*/
                static float dx = 0, dy = 0;
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                const float x = 10 + 0.4f*sinf(dx);
                const float y = 3.0f*cosf(dy);
                const float z = -15;

                float dmd[18], pyr[15];
                struct gjk_result gjk;

                diamond(dmd, x,y-0.5f,z, x,y+1,z, 0.5f);
                pyramid(pyr, 10.5f,-0.5f,-15.5f, 10.5f,1.0f,-15.5f, 1.0f);
                if (polyhedron_intersect_polyhedron(&gjk, dmd, 6, pyr, 5))
                    glColor3f(1,0,0);
                else glColor3f(1,1,1);

                glDiamondf(x,y-0.5f,z, x,y+1,z, 0.5f);
                glPyramidf(10.5f,-0.5f,-15.5f, 10.5f,1.0f,-15.5f, 1.0f);

                glBox(gjk.p0, 0.05f, 0.05f, 0.05f);
                glBox(gjk.p1, 0.05f, 0.05f, 0.05f);
                glLine(gjk.p0, gjk.p1);
            }
            {
                /* Polyhedron(Diamond)-AABB (GJK) intersection*/
                static float dx = 0, dy = 0;
                dx = dx + sys.time.delta * 2.0f;
                dy = dy + sys.time.delta * 0.8f;

                const float x = 20 + 0.4f*sinf(dx);
                const float y = 3.0f*cosf(dy);
                const float z = -15;

                float dmd[18];
                struct gjk_result gjk;
                diamond(dmd, x,y-0.5f,z, x,y+1,z, 0.5f);
                struct aabb a = aabbf(19.5f,-0.5f,-14.5f, 20.5f,0.5f,-15.5f);
                if (polyhedron_intersect_aabb(&gjk, dmd, 6, &a.min.x, &a.max.x))
                    glColor3f(1,0,0);
                else glColor3f(1,1,1);

                glDiamondf(x,y-0.5f,z, x,y+1,z, 0.5f);
                glBoxf(20,0,-15, 1,1,1);

                glBox(gjk.p0, 0.05f, 0.05f, 0.05f);
                glBox(gjk.p1, 0.05f, 0.05f, 0.05f);
                glLine(gjk.p0, gjk.p1);
            }
        }
        sys_push(&sys);
    }
    sys_shutdown(&sys);
    return 0;
}
