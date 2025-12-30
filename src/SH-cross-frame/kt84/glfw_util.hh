#pragma once
#include <string>
#include <functional>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "ScopeExit.hh"

namespace kt84 {
    namespace glfw_util {
        namespace callback_snippet {
            inline void error(int error_code, const char* description) {}
            inline void monitor(GLFWmonitor* monitor, int event) {
                bool is_connected = event == GLFW_CONNECTED;
                bool is_disconnected = event == GLFW_DISCONNECTED;
            }
            inline void windowpos(GLFWwindow* window, int xpos, int ypos) {}
            inline void windowsize(GLFWwindow* window, int width, int height) {}
            inline void windowclose(GLFWwindow* window) {}
            inline void windowrefresh(GLFWwindow* window) {}
            inline void windowfocus(GLFWwindow* window, int focused) {}
            inline void windowiconify(GLFWwindow* window, int iconified) {}
            inline void framebuffersize(GLFWwindow* window, int width, int height) {}
            inline void mousebutton(GLFWwindow* window, int button, int action, int mods) {
                bool is_left   = button == GLFW_MOUSE_BUTTON_LEFT;
                bool is_right  = button == GLFW_MOUSE_BUTTON_RIGHT;
                bool is_middle = button == GLFW_MOUSE_BUTTON_MIDDLE;
                bool is_pressed  = action == GLFW_PRESS;
                bool is_released = action == GLFW_RELEASE;
                bool is_shift = (mods & GLFW_MOD_SHIFT  ) != 0;
                bool is_ctrl  = (mods & GLFW_MOD_CONTROL) != 0;
                bool is_alt   = (mods & GLFW_MOD_ALT    ) != 0;
                bool is_super = (mods & GLFW_MOD_SUPER  ) != 0;
            }
            inline void cursorpos(GLFWwindow* window, double xpos, double ypos) {}
            inline void cursorenter(GLFWwindow* window, int entered) {}
            inline void scroll(GLFWwindow* window, double xoffset, double yoffset) {}
            inline void key(GLFWwindow* window, int key, int scancode, int action, int mods) {
                switch (key) {
                /* The unknown key */
                case GLFW_KEY_UNKNOWN:
                /* Printable keys */
                case GLFW_KEY_SPACE             :
                case GLFW_KEY_APOSTROPHE        :
                case GLFW_KEY_COMMA             :
                case GLFW_KEY_MINUS             :
                case GLFW_KEY_PERIOD            :
                case GLFW_KEY_SLASH             :
                case GLFW_KEY_0                 :
                case GLFW_KEY_1                 :
                case GLFW_KEY_2                 :
                case GLFW_KEY_3                 :
                case GLFW_KEY_4                 :
                case GLFW_KEY_5                 :
                case GLFW_KEY_6                 :
                case GLFW_KEY_7                 :
                case GLFW_KEY_8                 :
                case GLFW_KEY_9                 :
                case GLFW_KEY_SEMICOLON         :
                case GLFW_KEY_EQUAL             :
                case GLFW_KEY_A                 :
                case GLFW_KEY_B                 :
                case GLFW_KEY_C                 :
                case GLFW_KEY_D                 :
                case GLFW_KEY_E                 :
                case GLFW_KEY_F                 :
                case GLFW_KEY_G                 :
                case GLFW_KEY_H                 :
                case GLFW_KEY_I                 :
                case GLFW_KEY_J                 :
                case GLFW_KEY_K                 :
                case GLFW_KEY_L                 :
                case GLFW_KEY_M                 :
                case GLFW_KEY_N                 :
                case GLFW_KEY_O                 :
                case GLFW_KEY_P                 :
                case GLFW_KEY_Q                 :
                case GLFW_KEY_R                 :
                case GLFW_KEY_S                 :
                case GLFW_KEY_T                 :
                case GLFW_KEY_U                 :
                case GLFW_KEY_V                 :
                case GLFW_KEY_W                 :
                case GLFW_KEY_X                 :
                case GLFW_KEY_Y                 :
                case GLFW_KEY_Z                 :
                case GLFW_KEY_LEFT_BRACKET      :
                case GLFW_KEY_BACKSLASH         :
                case GLFW_KEY_RIGHT_BRACKET     :
                case GLFW_KEY_GRAVE_ACCENT      :
                case GLFW_KEY_WORLD_1           :
                case GLFW_KEY_WORLD_2           :
                /* Function keys */
                case GLFW_KEY_ESCAPE             :
                case GLFW_KEY_ENTER              :
                case GLFW_KEY_TAB                :
                case GLFW_KEY_BACKSPACE          :
                case GLFW_KEY_INSERT             :
                case GLFW_KEY_DELETE             :
                case GLFW_KEY_RIGHT              :
                case GLFW_KEY_LEFT               :
                case GLFW_KEY_DOWN               :
                case GLFW_KEY_UP                 :
                case GLFW_KEY_PAGE_UP            :
                case GLFW_KEY_PAGE_DOWN          :
                case GLFW_KEY_HOME               :
                case GLFW_KEY_END                :
                case GLFW_KEY_CAPS_LOCK          :
                case GLFW_KEY_SCROLL_LOCK        :
                case GLFW_KEY_NUM_LOCK           :
                case GLFW_KEY_PRINT_SCREEN       :
                case GLFW_KEY_PAUSE              :
                case GLFW_KEY_F1                 :
                case GLFW_KEY_F2                 :
                case GLFW_KEY_F3                 :
                case GLFW_KEY_F4                 :
                case GLFW_KEY_F5                 :
                case GLFW_KEY_F6                 :
                case GLFW_KEY_F7                 :
                case GLFW_KEY_F8                 :
                case GLFW_KEY_F9                 :
                case GLFW_KEY_F10                :
                case GLFW_KEY_F11                :
                case GLFW_KEY_F12                :
                case GLFW_KEY_F13                :
                case GLFW_KEY_F14                :
                case GLFW_KEY_F15                :
                case GLFW_KEY_F16                :
                case GLFW_KEY_F17                :
                case GLFW_KEY_F18                :
                case GLFW_KEY_F19                :
                case GLFW_KEY_F20                :
                case GLFW_KEY_F21                :
                case GLFW_KEY_F22                :
                case GLFW_KEY_F23                :
                case GLFW_KEY_F24                :
                case GLFW_KEY_F25                :
                case GLFW_KEY_KP_0               :
                case GLFW_KEY_KP_1               :
                case GLFW_KEY_KP_2               :
                case GLFW_KEY_KP_3               :
                case GLFW_KEY_KP_4               :
                case GLFW_KEY_KP_5               :
                case GLFW_KEY_KP_6               :
                case GLFW_KEY_KP_7               :
                case GLFW_KEY_KP_8               :
                case GLFW_KEY_KP_9               :
                case GLFW_KEY_KP_DECIMAL         :
                case GLFW_KEY_KP_DIVIDE          :
                case GLFW_KEY_KP_MULTIPLY        :
                case GLFW_KEY_KP_SUBTRACT        :
                case GLFW_KEY_KP_ADD             :
                case GLFW_KEY_KP_ENTER           :
                case GLFW_KEY_KP_EQUAL           :
                case GLFW_KEY_LEFT_SHIFT         :
                case GLFW_KEY_LEFT_CONTROL       :
                case GLFW_KEY_LEFT_ALT           :
                case GLFW_KEY_LEFT_SUPER         :
                case GLFW_KEY_RIGHT_SHIFT        :
                case GLFW_KEY_RIGHT_CONTROL      :
                case GLFW_KEY_RIGHT_ALT          :
                case GLFW_KEY_RIGHT_SUPER        :
                case GLFW_KEY_MENU               :
                    break;
                }
                bool is_pressed  = action == GLFW_PRESS;
                bool is_released = action == GLFW_RELEASE;
                bool is_repeated = action == GLFW_REPEAT;
            }
            inline void character(GLFWwindow* window, unsigned int codepoint) {}
            inline void charmods(GLFWwindow* window, unsigned int codepoint, int mods) {
                bool is_shift = (mods & GLFW_MOD_SHIFT  ) != 0;
                bool is_ctrl  = (mods & GLFW_MOD_CONTROL) != 0;
                bool is_alt   = (mods & GLFW_MOD_ALT    ) != 0;
                bool is_super = (mods & GLFW_MOD_SUPER  ) != 0;
            }
            inline void drop(GLFWwindow* window, int count, const char** names) {}
        }
        // utility for initialization
        struct InitConfig {
            struct Window {
                int width = 0;
                int height = 0;
                int xpos = -1;
                int ypos = -1;
                std::string title = "GLFW window";
            } window;
            struct Callback {
                decltype(callback_snippet::error          )* error           = nullptr;
                //decltype(callback_snippet::monitor        )* monitor         = nullptr;
                decltype(callback_snippet::windowpos      )* windowpos       = nullptr;
                decltype(callback_snippet::windowsize     )* windowsize      = nullptr;
                decltype(callback_snippet::windowclose    )* windowclose     = nullptr;
                decltype(callback_snippet::windowrefresh  )* windowrefresh   = nullptr;
                decltype(callback_snippet::windowfocus    )* windowfocus     = nullptr;
                decltype(callback_snippet::windowiconify  )* windowiconify   = nullptr;
                decltype(callback_snippet::framebuffersize)* framebuffersize = nullptr;
                decltype(callback_snippet::mousebutton    )* mousebutton     = nullptr;
                decltype(callback_snippet::cursorpos      )* cursorpos       = nullptr;
                decltype(callback_snippet::cursorenter    )* cursorenter     = nullptr;
                decltype(callback_snippet::scroll         )* scroll          = nullptr;
                decltype(callback_snippet::key            )* key             = nullptr;
                decltype(callback_snippet::character      )* character       = nullptr;
                decltype(callback_snippet::charmods       )* charmods        = nullptr;
                decltype(callback_snippet::drop           )* drop            = nullptr;
            } callback;
        };
        struct LoopControl {
            GLFWwindow* window = nullptr;
            std::function<void(void)> idle_func[16];
            bool poll_events = false;
        };
        inline LoopControl init(InitConfig config = {}) {
            glfwSetErrorCallback(config.callback.error);
            if (!glfwInit())
                return {};
            glfwWindowHint(GLFW_DEPTH_BITS, 16);
            //glfwSetMonitorCallback(config.callback.monitor);
            // create window with temporary width and height (in order to invoke framebuffersize callback)
            auto window = glfwCreateWindow(640, 480, config.window.title.c_str(), nullptr, nullptr);
            if (!window)
                return {};
            glfwMakeContextCurrent(window);
            // register callbacks to window
            glfwSetWindowPosCallback      (window, config.callback.windowpos      );
            glfwSetWindowSizeCallback     (window, config.callback.windowsize     );
            glfwSetWindowCloseCallback    (window, config.callback.windowclose    );
            glfwSetWindowRefreshCallback  (window, config.callback.windowrefresh  );
            glfwSetWindowFocusCallback    (window, config.callback.windowfocus    );
            glfwSetWindowIconifyCallback  (window, config.callback.windowiconify  );
            glfwSetFramebufferSizeCallback(window, config.callback.framebuffersize);
            glfwSetMouseButtonCallback    (window, config.callback.mousebutton    );
            glfwSetCursorPosCallback      (window, config.callback.cursorpos      );
            glfwSetCursorEnterCallback    (window, config.callback.cursorenter    );
            glfwSetScrollCallback         (window, config.callback.scroll         );
            glfwSetKeyCallback            (window, config.callback.key            );
            glfwSetCharCallback           (window, config.callback.character      );
            glfwSetCharModsCallback       (window, config.callback.charmods       );
            glfwSetDropCallback           (window, config.callback.drop           );
            // set window size and position
            auto video_mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
            if (config.window.width == 0 || config.window.height == 0) {
                config.window.width =  static_cast<int>(video_mode->width  * 0.8);
                config.window.height = static_cast<int>(video_mode->height * 0.8);
            }
            glfwSetWindowSize(window, config.window.width, config.window.height);
            if (config.window.xpos < 0 || config.window.ypos < 0) {
                config.window.xpos = (video_mode->width  - config.window.width ) / 2;
                config.window.ypos = (video_mode->height - config.window.height) / 2;
            }
            glfwSetWindowPos(window, config.window.xpos, config.window.ypos);
            glewInit();
            
            LoopControl control;
            control.window = window;
            return control;
        }
        inline void start_loop(const LoopControl& control) {
            while (!glfwWindowShouldClose(control.window)) {
                for (auto& f : control.idle_func)
                    if (f)
                        f();
                glfwSwapBuffers(control.window);
                (control.poll_events ? glfwPollEvents : glfwWaitEvents)();
            }
            glfwDestroyWindow(control.window);
            glfwTerminate();
        }
        // parsing int code
        struct ButtonFlag { bool left, right, middle; };
        inline ButtonFlag parseButton(int button) {
            return {
                button == GLFW_MOUSE_BUTTON_LEFT,
                button == GLFW_MOUSE_BUTTON_RIGHT,
                button == GLFW_MOUSE_BUTTON_MIDDLE
            };
        }
        struct ActionFlag { bool press, release, repeat; };
        inline ActionFlag parseAction(int action) {
            return {
                action == GLFW_PRESS,
                action == GLFW_RELEASE,
                action == GLFW_REPEAT
            };
        }
        struct ModFlag { bool shift, ctrl, alt, super; };
        inline ModFlag parseMods(int mods) {
            return {
                (mods & GLFW_MOD_SHIFT  ) != 0,
                (mods & GLFW_MOD_CONTROL) != 0,
                (mods & GLFW_MOD_ALT    ) != 0,
                (mods & GLFW_MOD_SUPER  ) != 0
            };
        }
        // set/get data
        struct CursorPos { int x, y; };
        inline CursorPos getCursorPos(GLFWwindow* window) {
            double x, y;
            glfwGetCursorPos(window, &x, &y);
            return { static_cast<int>(x), static_cast<int>(y) };
        }
        inline void setCursorPos(GLFWwindow* window, CursorPos pos) { glfwSetCursorPos(window, pos.x, pos.y); }
        struct WindowPos { int x, y; };
        inline WindowPos getWindowPos(GLFWwindow* window) {
            int x, y;
            glfwGetWindowPos(window, &x, &y);
            return { x, y };
        }
        inline void setWindowPos(GLFWwindow* window, WindowPos pos) { glfwSetWindowPos(window, pos.x, pos.y); }
        struct WindowSize { int width, height; };
        inline WindowSize getWindowSize(GLFWwindow* window) {
            int width, height;
            glfwGetWindowSize(window, &width, &height);
            return { width, height };
        }
        inline void setWindowSize(GLFWwindow* window, WindowSize size) { glfwSetWindowPos(window, size.width, size.height); }
#ifdef TW_INCLUDED
        inline bool TwEventKeyGLFW3(int key, int scancode, int action, int mods) {
            if (action != GLFW_PRESS)
                return false;
            auto modFlag = parseMods(mods);
            int tw_mods = 0;
            if (modFlag.alt  ) tw_mods += TW_KMOD_ALT;
            if (modFlag.ctrl ) tw_mods += TW_KMOD_CTRL;
            if (modFlag.shift) tw_mods += TW_KMOD_SHIFT;
            int tw_key = key;
            if (!modFlag.shift && 'A' <= key && key <= 'Z') tw_key += 'a' - 'A';
            if (255 < key) {
                tw_key =
                    key == GLFW_KEY_BACKSPACE ? tw_key = TW_KEY_BACKSPACE :
                    key == GLFW_KEY_TAB       ? tw_key = TW_KEY_TAB       :
                    key == GLFW_KEY_ENTER     ? tw_key = TW_KEY_RETURN    :
                    key == GLFW_KEY_PAUSE     ? tw_key = TW_KEY_PAUSE     :
                    key == GLFW_KEY_ESCAPE    ? tw_key = TW_KEY_ESCAPE    :
                    key == GLFW_KEY_DELETE    ? tw_key = TW_KEY_DELETE    :
                    key == GLFW_KEY_UP        ? tw_key = TW_KEY_UP        :
                    key == GLFW_KEY_DOWN      ? tw_key = TW_KEY_DOWN      :
                    key == GLFW_KEY_RIGHT     ? tw_key = TW_KEY_RIGHT     :
                    key == GLFW_KEY_LEFT      ? tw_key = TW_KEY_LEFT      :
                    key == GLFW_KEY_INSERT    ? tw_key = TW_KEY_INSERT    :
                    key == GLFW_KEY_HOME      ? tw_key = TW_KEY_HOME      :
                    key == GLFW_KEY_END       ? tw_key = TW_KEY_END       :
                    key == GLFW_KEY_PAGE_UP   ? tw_key = TW_KEY_PAGE_UP   :
                    key == GLFW_KEY_PAGE_DOWN ? tw_key = TW_KEY_PAGE_DOWN :
                    key == GLFW_KEY_F1        ? tw_key = TW_KEY_F1        :
                    key == GLFW_KEY_F2        ? tw_key = TW_KEY_F2        :
                    key == GLFW_KEY_F3        ? tw_key = TW_KEY_F3        :
                    key == GLFW_KEY_F4        ? tw_key = TW_KEY_F4        :
                    key == GLFW_KEY_F5        ? tw_key = TW_KEY_F5        :
                    key == GLFW_KEY_F6        ? tw_key = TW_KEY_F6        :
                    key == GLFW_KEY_F7        ? tw_key = TW_KEY_F7        :
                    key == GLFW_KEY_F8        ? tw_key = TW_KEY_F8        :
                    key == GLFW_KEY_F9        ? tw_key = TW_KEY_F9        :
                    key == GLFW_KEY_F10       ? tw_key = TW_KEY_F10       :
                    key == GLFW_KEY_F11       ? tw_key = TW_KEY_F11       :
                    key == GLFW_KEY_F12       ? tw_key = TW_KEY_F12       :
                    key == GLFW_KEY_F13       ? tw_key = TW_KEY_F13       :
                    key == GLFW_KEY_F14       ? tw_key = TW_KEY_F14       :
                    key == GLFW_KEY_F15       ? tw_key = TW_KEY_F15       :
                    0;
            }
            return tw_key && TwKeyPressed(tw_key, tw_mods);
        }
#endif
    }
}
