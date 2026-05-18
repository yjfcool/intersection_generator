#pragma once
/**
 * 内嵌轻量级 JSON 解析/序列化库
 * 接口兼容 nlohmann/json 常用子集
 * 支持: object, array, string, number(int/float), bool, null
 */
#include <string>
#include <vector>
#include <map>
#include <stdexcept>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <cstring>

namespace nlohmann {

class json {
public:
    enum class value_t { null_t, bool_t, int_t, float_t, string_t, array_t, object_t };

    using object_t = std::map<std::string, json>;
    using array_t  = std::vector<json>;

private:
    value_t     vt_  = value_t::null_t;
    bool        b_   = false;
    long long   i_   = 0;
    double      d_   = 0.0;
    std::string s_;
    array_t     arr_;
    object_t    obj_;

public:
    // ---------- 构造 ----------
    json()                     : vt_(value_t::null_t)   {}
    json(std::nullptr_t)       : vt_(value_t::null_t)   {}
    json(bool v)               : vt_(value_t::bool_t),  b_(v) {}
    json(int v)                : vt_(value_t::int_t),   i_(v) {}
    json(long v)               : vt_(value_t::int_t),   i_(v) {}
    json(long long v)          : vt_(value_t::int_t),   i_(v) {}
    json(unsigned v)           : vt_(value_t::int_t),   i_((long long)v) {}
    json(float v)              : vt_(value_t::float_t), d_(v) {}
    json(double v)             : vt_(value_t::float_t), d_(v) {}
    json(const std::string& v) : vt_(value_t::string_t),s_(v) {}
    json(std::string&& v)      : vt_(value_t::string_t),s_(std::move(v)) {}
    json(const char* v)        : vt_(value_t::string_t),s_(v) {}
    json(const array_t& v)     : vt_(value_t::array_t), arr_(v) {}
    json(const object_t& v)    : vt_(value_t::object_t),obj_(v) {}

    // 初始化列表（数组语法 {1,2,3}）
    json(std::initializer_list<json> il) {
        // 若每个元素是 [key, value] 对 → object，否则 → array
        bool is_obj = true;
        for (auto& el : il) {
            if (!el.is_array() || el.size() != 2 || !el[0].is_string()) {
                is_obj = false; break;
            }
        }
        if (is_obj && il.size()>0) {
            vt_ = value_t::object_t;
            for (auto& el : il) obj_[el[0].get<std::string>()] = el[1];
        } else {
            vt_ = value_t::array_t;
            for (auto& el : il) arr_.push_back(el);
        }
    }

    static json object() { json j; j.vt_=value_t::object_t; return j; }
    static json array()  { json j; j.vt_=value_t::array_t;  return j; }

    // ---------- 类型查询 ----------
    bool is_null()    const { return vt_==value_t::null_t; }
    bool is_boolean() const { return vt_==value_t::bool_t; }
    bool is_number()  const { return vt_==value_t::int_t || vt_==value_t::float_t; }
    bool is_number_integer() const { return vt_==value_t::int_t; }
    bool is_number_float()   const { return vt_==value_t::float_t; }
    bool is_string()  const { return vt_==value_t::string_t; }
    bool is_array()   const { return vt_==value_t::array_t; }
    bool is_object()  const { return vt_==value_t::object_t; }

    // ---------- get<T> ----------
    template<typename T>
    T get() const {
        if constexpr (std::is_same_v<T, bool>) {
            if (vt_==value_t::bool_t)  return b_;
            if (vt_==value_t::int_t)   return i_ != 0;
            return false;
        } else if constexpr (std::is_same_v<T, int>) {
            if (vt_==value_t::int_t)   return (int)i_;
            if (vt_==value_t::float_t) return (int)d_;
            return 0;
        } else if constexpr (std::is_same_v<T, long long>) {
            if (vt_==value_t::int_t)   return i_;
            if (vt_==value_t::float_t) return (long long)d_;
            return 0LL;
        } else if constexpr (std::is_same_v<T, double>) {
            if (vt_==value_t::float_t) return d_;
            if (vt_==value_t::int_t)   return (double)i_;
            return 0.0;
        } else if constexpr (std::is_same_v<T, std::string>) {
            if (vt_==value_t::string_t) return s_;
            if (vt_==value_t::int_t)    return std::to_string(i_);
            if (vt_==value_t::float_t)  { std::ostringstream o; o<<d_; return o.str(); }
            if (vt_==value_t::bool_t)   return b_?"true":"false";
            return "";
        } else {
            return T{};
        }
    }

    // ---------- value(key, default) ----------
    template<typename T>
    T value(const std::string& key, const T& def) const {
        if (!is_object()) return def;
        auto it = obj_.find(key);
        if (it == obj_.end()) return def;
        return it->second.get<T>();
    }

    // ---------- 下标访问 ----------
    const json& operator[](const std::string& key) const {
        auto it = obj_.find(key);
        if (it == obj_.end()) throw std::out_of_range("key not found: " + key);
        return it->second;
    }
    json& operator[](const std::string& key) {
        if (vt_ == value_t::null_t) vt_ = value_t::object_t;
        return obj_[key];
    }
    const json& operator[](size_t idx) const { return arr_.at(idx); }
    json& operator[](size_t idx)       { return arr_.at(idx); }
    const json& operator[](int idx)    const { return arr_.at((size_t)idx); }
    json& operator[](int idx)          { return arr_.at((size_t)idx); }

    bool contains(const std::string& key) const {
        return is_object() && obj_.count(key) > 0;
    }

    // ---------- 大小 ----------
    size_t size() const {
        if (vt_ == value_t::array_t)  return arr_.size();
        if (vt_ == value_t::object_t) return obj_.size();
        return 0;
    }
    bool empty() const { return size() == 0; }

    // ---------- 数组操作 ----------
    void push_back(const json& v) {
        if (vt_ == value_t::null_t) vt_ = value_t::array_t;
        arr_.push_back(v);
    }
    json& back() { return arr_.back(); }

    // ---------- 迭代（数组） ----------
    typename array_t::iterator       begin()       { return arr_.begin(); }
    typename array_t::iterator       end()         { return arr_.end();   }
    typename array_t::const_iterator begin() const { return arr_.begin(); }
    typename array_t::const_iterator end()   const { return arr_.end();   }

    // ---------- items()（对象迭代） ----------
    struct ItemsProxy {
        const object_t& o;
        typename object_t::const_iterator begin() const { return o.begin(); }
        typename object_t::const_iterator end()   const { return o.end();   }
    };
    ItemsProxy items() const { return {obj_}; }

    // ---------- dump / serialize ----------
    std::string dump(int indent = -1, int depth = 0) const {
        std::string ind  = (indent >= 0) ? std::string((depth+1)*indent, ' ') : "";
        std::string ind0 = (indent >= 0) ? std::string(depth*indent,     ' ') : "";
        std::string nl   = (indent >= 0) ? "\n" : "";
        std::string sp   = (indent >= 0) ? " "  : "";

        switch (vt_) {
            case value_t::null_t:   return "null";
            case value_t::bool_t:   return b_ ? "true" : "false";
            case value_t::int_t:    return std::to_string(i_);
            case value_t::float_t: {
                std::ostringstream oss;
                oss << std::setprecision(10) << d_;
                std::string s = oss.str();
                // 确保含小数点
                if (s.find('.')==std::string::npos && s.find('e')==std::string::npos)
                    s += ".0";
                return s;
            }
            case value_t::string_t: return escapeStr(s_);
            case value_t::array_t: {
                if (arr_.empty()) return "[]";
                std::string r = "[" + nl;
                for (size_t i = 0; i < arr_.size(); ++i) {
                    r += ind + arr_[i].dump(indent, depth+1);
                    if (i+1 < arr_.size()) r += ",";
                    r += nl;
                }
                return r + ind0 + "]";
            }
            case value_t::object_t: {
                if (obj_.empty()) return "{}";
                std::string r = "{" + nl;
                size_t i = 0;
                for (auto& [k, v] : obj_) {
                    r += ind + escapeStr(k) + ":" + sp + v.dump(indent, depth+1);
                    if (i+1 < obj_.size()) r += ",";
                    r += nl;
                    ++i;
                }
                return r + ind0 + "}";
            }
        }
        return "null";
    }

    // ---------- 流操作符 ----------
    friend std::istream& operator>>(std::istream& is, json& j) {
        std::string src((std::istreambuf_iterator<char>(is)),
                         std::istreambuf_iterator<char>());
        size_t pos = 0;
        skipWS(src, pos);
        j = parseValue(src, pos);
        return is;
    }
    friend std::ostream& operator<<(std::ostream& os, const json& j) {
        os << j.dump();
        return os;
    }

    // ---------- 静态 parse ----------
    static json parse(const std::string& src) {
        size_t pos = 0;
        skipWS(src, pos);
        return parseValue(src, pos);
    }

private:
    // ---- 序列化辅助 ----
    static std::string escapeStr(const std::string& s) {
        std::string r = "\"";
        for (unsigned char c : s) {
            switch (c) {
                case '"':  r += "\\\""; break;
                case '\\': r += "\\\\"; break;
                case '\n': r += "\\n";  break;
                case '\r': r += "\\r";  break;
                case '\t': r += "\\t";  break;
                default:
                    if (c < 0x20) {
                        char buf[8]; snprintf(buf,8,"\\u%04x",(int)c);
                        r += buf;
                    } else r += (char)c;
            }
        }
        return r + "\"";
    }

    // ---- 解析辅助 ----
    static void skipWS(const std::string& s, size_t& p) {
        while (p < s.size() && (s[p]==' '||s[p]=='\t'||s[p]=='\n'||s[p]=='\r'||s[p]==0xef||s[p]==0xbb||s[p]==0xbf))
            ++p;
    }

    static json parseValue(const std::string& s, size_t& p) {
        skipWS(s, p);
        if (p >= s.size()) throw std::runtime_error("Unexpected EOF in JSON");
        char c = s[p];
        if (c == '{') return parseObject(s, p);
        if (c == '[') return parseArray(s, p);
        if (c == '"') return json(parseString(s, p));
        if (c == 't') { p += 4; return json(true);    }
        if (c == 'f') { p += 5; return json(false);   }
        if (c == 'n') { p += 4; return json(nullptr); }
        return parseNumber(s, p);
    }

    static json parseObject(const std::string& s, size_t& p) {
        json obj; obj.vt_ = value_t::object_t;
        ++p; // skip '{'
        skipWS(s, p);
        if (p < s.size() && s[p] == '}') { ++p; return obj; }
        while (p < s.size()) {
            skipWS(s, p);
            std::string key = parseString(s, p);
            skipWS(s, p);
            if (p >= s.size() || s[p] != ':')
                throw std::runtime_error("Expected ':' in JSON object");
            ++p;
            skipWS(s, p);
            obj.obj_[key] = parseValue(s, p);
            skipWS(s, p);
            if (p < s.size() && s[p] == ',') { ++p; continue; }
            if (p < s.size() && s[p] == '}') { ++p; break; }
            throw std::runtime_error("Expected ',' or '}' in JSON object");
        }
        return obj;
    }

    static json parseArray(const std::string& s, size_t& p) {
        json arr; arr.vt_ = value_t::array_t;
        ++p; // skip '['
        skipWS(s, p);
        if (p < s.size() && s[p] == ']') { ++p; return arr; }
        while (p < s.size()) {
            skipWS(s, p);
            arr.arr_.push_back(parseValue(s, p));
            skipWS(s, p);
            if (p < s.size() && s[p] == ',') { ++p; continue; }
            if (p < s.size() && s[p] == ']') { ++p; break; }
            throw std::runtime_error("Expected ',' or ']' in JSON array");
        }
        return arr;
    }

    static std::string parseString(const std::string& s, size_t& p) {
        if (p >= s.size() || s[p] != '"')
            throw std::runtime_error("Expected '\"' for JSON string");
        ++p;
        std::string r;
        while (p < s.size() && s[p] != '"') {
            if (s[p] == '\\') {
                ++p;
                if (p >= s.size()) break;
                switch (s[p]) {
                    case '"':  r += '"';  break;
                    case '\\': r += '\\'; break;
                    case '/':  r += '/';  break;
                    case 'n':  r += '\n'; break;
                    case 'r':  r += '\r'; break;
                    case 't':  r += '\t'; break;
                    case 'b':  r += '\b'; break;
                    case 'f':  r += '\f'; break;
                    case 'u': {
                        if (p+4 < s.size()) {
                            // 简单处理ASCII范围
                            std::string hex = s.substr(p+1, 4);
                            unsigned int cp = 0;
                            try { cp = std::stoul(hex, nullptr, 16); } catch(...) {}
                            if (cp < 128) r += (char)cp;
                            else r += '?';
                            p += 4;
                        }
                        break;
                    }
                    default: r += s[p];
                }
            } else {
                r += s[p];
            }
            ++p;
        }
        if (p < s.size()) ++p; // skip closing '"'
        return r;
    }

    static json parseNumber(const std::string& s, size_t& p) {
        size_t start = p;
        bool isFloat = false;
        if (p < s.size() && s[p] == '-') ++p;
        while (p < s.size() && s[p] >= '0' && s[p] <= '9') ++p;
        if (p < s.size() && s[p] == '.') {
            isFloat = true; ++p;
            while (p < s.size() && s[p] >= '0' && s[p] <= '9') ++p;
        }
        if (p < s.size() && (s[p] == 'e' || s[p] == 'E')) {
            isFloat = true; ++p;
            if (p < s.size() && (s[p] == '+' || s[p] == '-')) ++p;
            while (p < s.size() && s[p] >= '0' && s[p] <= '9') ++p;
        }
        std::string num = s.substr(start, p - start);
        if (num.empty()) throw std::runtime_error("Invalid number in JSON");
        try {
            if (isFloat) return json(std::stod(num));
            else         return json((long long)std::stoll(num));
        } catch (...) {
            throw std::runtime_error("Number parse error: " + num);
        }
    }
};


} // namespace nlohmann
