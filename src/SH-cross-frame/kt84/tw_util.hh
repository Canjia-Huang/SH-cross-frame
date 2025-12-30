#pragma once
#include <cassert>
#include <functional>
#include <AntTweakBar.h>
#include "string_util.hh"

namespace kt84 {
    namespace tw_util {
        // AddButton |
        //-----------+
        namespace internal {
            inline void TW_CALL AddButton_sub(void *clientData) {
                static_cast<std::function<void(void)>*>(clientData)->operator()();
            }
        }
        inline int AddButton(TwBar *bar, const char *name, std::function<void(void)> func, const char *def) {
            static const int buffer_size = 1000;
            static std::function<void(void)> buffer[buffer_size];
            static int counter = 0;
            assert(counter < buffer_size);
            buffer[counter] = func;
            return TwAddButton(bar, name, internal::AddButton_sub, &buffer[counter++], def);
        }
        // AddVarCB |
        //----------+
        namespace internal {
            template <typename T>
            struct FuncPair {
                std::function<void(const T&)> set_func;
                std::function<void(      T&)> get_func;
            };
            template <typename T>
            inline void TW_CALL AddVarCB_sub_set(const void *value, void *clientData) {
                static_cast<FuncPair<T>*>(clientData)->set_func(*static_cast<const T*>(value));
            }
            template <typename T>
            inline void TW_CALL AddVarCB_sub_get(void *value, void *clientData) {
                static_cast<FuncPair<T>*>(clientData)->get_func(*static_cast<      T*>(value));
            }
            template <>
            inline void TW_CALL AddVarCB_sub_get<std::string>(void *value, void *clientData) {
                using T = std::string;
                T value_copy;
                static_cast<FuncPair<T>*>(clientData)->get_func(value_copy);
                TwCopyStdStringToLibrary(*static_cast<T*>(value), value_copy);
            }
        }
        template <typename T>
        inline int AddVarCB(TwBar *bar, const char *name, TwType type, std::function<void(const T&)> set_func, std::function<void(T&)> get_func, const char *def) {
            static const int buffer_size = 1000;
            static internal::FuncPair<T> buffer[buffer_size];
            static int counter = 0;
            assert(counter < buffer_size);
            buffer[counter] = { set_func, get_func };
            return TwAddVarCB(bar, name, type, internal::AddVarCB_sub_set<T>, internal::AddVarCB_sub_get<T>, &buffer[counter++], def);
        }
        template <>
        inline int AddVarCB<std::string>(TwBar *bar, const char *name, TwType type, std::function<void(const std::string&)> set_func, std::function<void(std::string&)> get_func, const char *def) {
            // automatic registration of string copy func
            using T = std::string;
            static bool flag = false;
            if (!flag) {
                TwCopyStdStringToClientFunc([](T& destinationClientString, const T& sourceLibraryString) {
                    destinationClientString = sourceLibraryString;
                });
                flag = true;
            }
            static const int buffer_size = 1000;
            static internal::FuncPair<T> buffer[buffer_size];
            static int counter = 0;
            assert(counter < buffer_size);
            buffer[counter] = { set_func, get_func };
            return TwAddVarCB(bar, name, type, internal::AddVarCB_sub_set<T>, internal::AddVarCB_sub_get<T>, &buffer[counter++], def);
        }
        // AddVarCB_default: default get func and custom func after default set func |
        //---------------------------------------------------------------------------+
        template <typename T>
        inline int AddVarCB_default(TwBar *bar, const char *name, TwType type, T& client_value, std::function<void(void)> func_after_set, const char *def) {
            return AddVarCB<T>(bar, name, type, [&, func_after_set](const T& value) { client_value = value; func_after_set(); },
                                                [&                ](      T& value) { value = client_value;                   }, def);
        }
        // API with TW_TYPE_*** omitted for certain types
        namespace internal {
            template <typename T> struct ETwTypeT;
            template <> struct ETwTypeT<bool       > { static const ETwType Value = TW_TYPE_BOOLCPP  ; };
            template <> struct ETwTypeT<int        > { static const ETwType Value = TW_TYPE_INT32    ; };
            template <> struct ETwTypeT<float      > { static const ETwType Value = TW_TYPE_FLOAT    ; };
            template <> struct ETwTypeT<double     > { static const ETwType Value = TW_TYPE_DOUBLE   ; };
            template <> struct ETwTypeT<std::string> { static const ETwType Value = TW_TYPE_STDSTRING; };
        }
        template <typename T>
        inline int AddVarCB(TwBar *bar, const char *name, std::function<void(const T&)> set_func, std::function<void(T&)> get_func, const char *def) {
            return AddVarCB(bar, name, internal::ETwTypeT<T>::Value, set_func, get_func, def);
        }
        template <typename T>
        inline int AddVarCB_default(TwBar *bar, const char *name, T& client_value, std::function<void(void)> func_after_set, const char *def) {
            return AddVarCB_default(bar, name, internal::ETwTypeT<T>::Value, client_value, func_after_set, def);
        }
        // little shortcut for for custom enums
        template <typename T>
        inline int AddVarCB(TwBar *bar, const char *name, const char *enum_type_name, std::function<void(const T&)> set_func, std::function<void(T&)> get_func, const char *def) {
            return AddVarCB(bar, name, TwDefineEnum(enum_type_name, 0, 0), set_func, get_func, def);
        }
        template <typename T>
        inline int AddVarCB_default(TwBar *bar, const char *name, const char *enum_type_name, T& client_value, std::function<void(void)> func_after_set, const char *def) {
            return AddVarCB_default(bar, name, TwDefineEnum(enum_type_name, 0, 0), client_value, func_after_set, def);
        }
        // set bar parameter |
        //-------------------+
        namespace internal {
            template <typename T>
            inline void set_bar_param(const char* bar_name, const char* param_name, T param) {
                TwDefine((string_util::cat(bar_name) + ' ' + param_name + '=' + param).c_str());
            }
            template <typename T>
            inline void set_bar_param(const char* bar_name, const char* param_name, T param1, T param2) {
                TwDefine((string_util::cat(bar_name) + ' ' + param_name + "='" + param1 + ' ' + param2 + "'").c_str());
            }
            template <typename T>
            inline void set_bar_param(const char* bar_name, const char* param_name, T param1, T param2, T param3) {
                TwDefine((string_util::cat(bar_name) + ' ' + param_name + "='" + param1 + ' ' + param2 + ' ' + param3 + "'").c_str());
            }
            template <typename T>
            inline void set_bar_param(TwBar* bar, const char* param_name, T param) {
                const char* bar_name = TwGetBarName(bar);
                set_bar_param(bar_name, param_name, param);
            }
            template <typename T>
            inline void set_bar_param(TwBar* bar, const char* param_name, T param1, T param2) {
                const char* bar_name = TwGetBarName(bar);
                set_bar_param(bar_name, param_name, param1, param2);
            }
            template <typename T>
            inline void set_bar_param(TwBar* bar, const char* param_name, T param1, T param2, T param3) {
                const char* bar_name = TwGetBarName(bar);
                set_bar_param(bar_name, param_name, param1, param2, param3);
            }
        }
        inline void set_bar_label(TwBar* bar, const char* string) { internal::set_bar_param(bar, "label", string); }
        inline void set_bar_help(TwBar* bar, const char* string) { internal::set_bar_param(bar, "help", string);}
        inline void set_bar_color(TwBar* bar, unsigned char red, unsigned char green, unsigned char blue) { internal::set_bar_param(bar, "color", red, green, blue); }
        inline void set_bar_alpha(TwBar* bar, unsigned char alpha) { internal::set_bar_param(bar, "alpha", alpha); }
        inline void set_bar_text_light(TwBar* bar) { internal::set_bar_param(bar, "text", "light"); }
        inline void set_bar_text_dark (TwBar* bar) { internal::set_bar_param(bar, "text", "dark"); }
        inline void set_bar_position(TwBar* bar, unsigned int x, unsigned int y) { internal::set_bar_param(bar, "position", x, y); }
        inline void set_bar_size(TwBar* bar, unsigned int sx, unsigned int sy) { internal::set_bar_param(bar, "size", sx, sy); }
        inline void set_bar_valueswidth    (TwBar* bar, unsigned int w) { internal::set_bar_param(bar, "valueswidth", w); }
        inline void set_bar_valueswidth_fit(TwBar* bar)                 { internal::set_bar_param(bar, "valueswidth", "fit"); }
        inline void set_bar_refresh(TwBar* bar, double r) { internal::set_bar_param(bar, "refresh", r); }
        inline void set_bar_visible(TwBar* bar, bool b) { internal::set_bar_param(bar, "visible", b); }
        inline void set_bar_iconified(TwBar* bar, bool b) { internal::set_bar_param(bar, "iconified", b); }
        inline void set_bar_iconpos_bottomleft () { internal::set_bar_param("GLOBAL", "iconpos", "bl"); }
        inline void set_bar_iconpos_bottomright() { internal::set_bar_param("GLOBAL", "iconpos", "br"); }
        inline void set_bar_iconpos_topleft    () { internal::set_bar_param("GLOBAL", "iconpos", "tl"); }
        inline void set_bar_iconpos_topright   () { internal::set_bar_param("GLOBAL", "iconpos", "tr"); }
        inline void set_bar_iconalign_vertical  () { internal::set_bar_param("GLOBAL", "iconalign", "vertical"); }
        inline void set_bar_iconalign_horizontal() { internal::set_bar_param("GLOBAL", "iconalign", "horizontal"); }
        inline void set_bar_iconmargin(unsigned int x, unsigned int y) { internal::set_bar_param("GLOBAL", "iconmargin", x, y); }
        inline void set_bar_iconifiable(TwBar* bar, bool b) { internal::set_bar_param(bar, "iconifiable", b); }
        inline void set_bar_movable(TwBar* bar, bool b) { internal::set_bar_param(bar, "movable", b); }
        inline void set_bar_resizable(TwBar* bar, bool b) { internal::set_bar_param(bar, "resizable", b); }
        inline void set_bar_fontsize_small () { internal::set_bar_param("GLOBAL", "fontsize", 1); }
        inline void set_bar_fontsize_medium() { internal::set_bar_param("GLOBAL", "fontsize", 2); }
        inline void set_bar_fontsize_large () { internal::set_bar_param("GLOBAL", "fontsize", 3); }
        inline void set_bar_fontstyle_default() { internal::set_bar_param("GLOBAL", "fontstyle", "default"); }
        inline void set_bar_fontstyle_fixed  () { internal::set_bar_param("GLOBAL", "fontstyle", "fixed"); }
        inline void set_bar_fontresizable(bool b) { internal::set_bar_param("GLOBAL", "fontresizable", b); }
        inline void set_bar_fontscaling(double s) { internal::set_bar_param("GLOBAL", "fontscaling", s); }
        inline void set_bar_alwaystop(TwBar* bar, bool b) { internal::set_bar_param(bar, "alwaystop", b); }
        inline void set_bar_alwaysbottom(TwBar* bar, bool b) { internal::set_bar_param(bar, "alwaysbottom", b); }
        inline void set_bar_contained(bool b) { internal::set_bar_param("GLOBAL", "contained", b); }
        inline void set_bar_overlap(bool b) { internal::set_bar_param("GLOBAL", "overlap", b); }
        inline void set_bar_buttonalign_left  () { internal::set_bar_param("GLOBAL", "buttonalign", "left"); }
        inline void set_bar_buttonalign_center() { internal::set_bar_param("GLOBAL", "buttonalign", "center"); }
        inline void set_bar_buttonalign_right () { internal::set_bar_param("GLOBAL", "buttonalign", "right"); }
        // set variable parameter |
        //------------------------+
        namespace internal {
            template <typename T>
            inline void set_var_param(TwBar* bar, const char* var_name, const char* param_name, T param) {
                const char* bar_name = TwGetBarName(bar);
                TwDefine((string_util::cat(bar_name) + '/' + var_name + ' ' + param_name + '=' + param).c_str());
            }
            template <typename T>
            inline void set_var_param(TwBar* bar, const char* var_name, const char* param_name, T param1, T param2, T param3) {
                const char* bar_name = TwGetBarName(bar);
                TwDefine((string_util::cat(bar_name) + '/' + var_name + ' ' + param_name + "='" + param1 + ' ' + param2 + ' ' + param3 + "'").c_str());
            }
        }
        inline void set_var_label(TwBar* bar, const char* var_name, const char* string) { internal::set_var_param(bar, var_name, "label", string); }
        inline void set_var_help(TwBar* bar, const char* var_name, const char* string) { internal::set_var_param(bar, var_name, "help", string); }
        inline void set_var_group(TwBar* bar, const char* var_name, const char* groupname) { internal::set_var_param(bar, var_name, "group", groupname); }
        inline void set_var_visible(TwBar* bar, const char* var_name, bool b) { internal::set_var_param(bar, var_name, "visible", b); }
        inline void set_var_readonly(TwBar* bar, const char* var_name, bool b) { internal::set_var_param(bar, var_name, "readonly", b); }
        inline void set_var_min(TwBar* bar, const char* var_name, double value) { internal::set_var_param(bar, var_name, "min", value); }
        inline void set_var_max(TwBar* bar, const char* var_name, double value) { internal::set_var_param(bar, var_name, "max", value); }
        inline void set_var_step(TwBar* bar, const char* var_name, double value) { internal::set_var_param(bar, var_name, "step", value); }
        inline void set_var_precision(TwBar* bar, const char* var_name, int value) { internal::set_var_param(bar, var_name, "precision", value); }
        inline void set_var_hexa(TwBar* bar, const char* var_name, bool b) { internal::set_var_param(bar, var_name, "hexa", b); }
        inline void set_var_key(TwBar* bar, const char* var_name, const char* keyshortcut) { internal::set_var_param(bar, var_name, "key", keyshortcut); }
        inline void set_var_keyincr(TwBar* bar, const char* var_name, const char* keyshortcut) { internal::set_var_param(bar, var_name, "keyincr", keyshortcut); }
        inline void set_var_keydecr(TwBar* bar, const char* var_name, const char* keyshortcut) { internal::set_var_param(bar, var_name, "keydecr", keyshortcut); }
        inline void set_var_true(TwBar* bar, const char* var_name, const char* string) { internal::set_var_param(bar, var_name, "true", string); }
        inline void set_var_false(TwBar* bar, const char* var_name, const char* string) { internal::set_var_param(bar, var_name, "false", string); }
        inline void set_var_opened(TwBar* bar, const char* var_name, bool b) { internal::set_var_param(bar, var_name, "opened", b); }
        //inline void set_var_enum(TwBar* bar, const char* var_name, ) { internal::set_var_param(bar, var_name, "enum", ); }
        inline void set_var_coloralpha(TwBar* bar, const char* var_name, bool b) { internal::set_var_param(bar, var_name, "coloralpha", b); }
        inline void set_var_colororder_rgba(TwBar* bar, const char* var_name) { internal::set_var_param(bar, var_name, "colororder", "rgba"); }
        inline void set_var_colororder_argb(TwBar* bar, const char* var_name) { internal::set_var_param(bar, var_name, "colororder", "argb"); }
        inline void set_var_colormode_rgb(TwBar* bar, const char* var_name) { internal::set_var_param(bar, var_name, "colormode", "rgb"); }
        inline void set_var_colormode_hls(TwBar* bar, const char* var_name) { internal::set_var_param(bar, var_name, "colormode", "hls"); }
        inline void set_var_arrow(TwBar* bar, const char* var_name, double x, double y, double z) { internal::set_var_param(bar, var_name, "arrow", x, y, z); }
        inline void set_var_arrow_restore(TwBar* bar, const char* var_name) { internal::set_var_param(bar, var_name, "arrow", 0); }
        inline void set_var_arrowcolor(TwBar* bar, const char* var_name, unsigned char r, unsigned char g, unsigned char b) { internal::set_var_param(bar, var_name, "arrowcolor", r, g, b); }
        inline void set_var_axisx(TwBar* bar, const char* var_name, const char* n) { internal::set_var_param(bar, var_name, "axisx", n); }
        inline void set_var_axisy(TwBar* bar, const char* var_name, const char* n) { internal::set_var_param(bar, var_name, "axisy", n); }
        inline void set_var_axisz(TwBar* bar, const char* var_name, const char* n) { internal::set_var_param(bar, var_name, "axisz", n); }
        inline void set_var_showval(TwBar* bar, const char* var_name, bool b) { internal::set_var_param(bar, var_name, "showval", b); }
    }
}
