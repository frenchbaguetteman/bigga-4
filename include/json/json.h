/**
 * @file json.h
 * Minimal embedded-friendly JSON helpers.
 *
 * Provides:
 *  • JsonValue   — a lightweight tagged-union JSON DOM
 *  • JsonBuilder — a streaming JSON serialiser (for telemetry)
 *  • parseJson() — a recursive-descent parser for embedded assets
 */
#pragma once

#include <string>
#include <vector>
#include <map>
#include <variant>
#include <sstream>
#include <cctype>
#include <cstdlib>
#include <optional>

// ── JsonValue ───────────────────────────────────────────────────────────────

class JsonValue {
public:
    using Object = std::map<std::string, JsonValue>;
    using Array  = std::vector<JsonValue>;

    enum Type { Null, Number, String, Bool, ArrayT, ObjectT };

    JsonValue() : m_type(Null) {}
    explicit JsonValue(double v) : m_type(Number), m_number(v) {}
    explicit JsonValue(const std::string& s) : m_type(String), m_string(s) {}
    explicit JsonValue(bool b) : m_type(Bool), m_bool(b) {}
    explicit JsonValue(const Array& a) : m_type(ArrayT), m_array(a) {}
    explicit JsonValue(const Object& o) : m_type(ObjectT), m_object(o) {}

    Type type() const { return m_type; }

    double              asNumber() const { return m_number; }
    float               asFloat()  const { return static_cast<float>(m_number); }
    const std::string&  asString() const { return m_string; }
    bool                asBool()   const { return m_bool; }
    const Array&        asArray()  const { return m_array; }
    const Object&       asObject() const { return m_object; }

    bool has(const std::string& key) const {
        return m_type == ObjectT && m_object.count(key);
    }
    const JsonValue& operator[](const std::string& key) const {
        return m_object.at(key);
    }
    const JsonValue& operator[](size_t idx) const {
        return m_array.at(idx);
    }
    size_t size() const {
        if (m_type == ArrayT)  return m_array.size();
        if (m_type == ObjectT) return m_object.size();
        return 0;
    }

private:
    Type        m_type   = Null;
    double      m_number = 0.0;
    std::string m_string;
    bool        m_bool   = false;
    Array       m_array;
    Object      m_object;
};

// ── Minimal recursive-descent parser ────────────────────────────────────────

namespace json_detail {
    inline void skipWhitespace(const char*& p) {
        while (*p && std::isspace(static_cast<unsigned char>(*p))) ++p;
    }

    inline JsonValue parseValue(const char*& p);

    inline std::string parseString(const char*& p) {
        ++p; // skip opening "
        std::string s;
        while (*p && *p != '"') {
            if (*p == '\\') { ++p; s += *p++; continue; }
            s += *p++;
        }
        if (*p == '"') ++p;
        return s;
    }

    inline JsonValue parseNumber(const char*& p) {
        char* end = nullptr;
        double v = std::strtod(p, &end);
        p = end;
        return JsonValue(v);
    }

    inline JsonValue parseArray(const char*& p) {
        ++p; // skip [
        JsonValue::Array arr;
        skipWhitespace(p);
        if (*p == ']') { ++p; return JsonValue(arr); }
        while (true) {
            arr.push_back(parseValue(p));
            skipWhitespace(p);
            if (*p == ',') { ++p; skipWhitespace(p); continue; }
            if (*p == ']') { ++p; break; }
            break; // malformed
        }
        return JsonValue(arr);
    }

    inline JsonValue parseObject(const char*& p) {
        ++p; // skip {
        JsonValue::Object obj;
        skipWhitespace(p);
        if (*p == '}') { ++p; return JsonValue(obj); }
        while (true) {
            skipWhitespace(p);
            std::string key = parseString(p);
            skipWhitespace(p);
            if (*p == ':') ++p;
            skipWhitespace(p);
            obj[key] = parseValue(p);
            skipWhitespace(p);
            if (*p == ',') { ++p; continue; }
            if (*p == '}') { ++p; break; }
            break;
        }
        return JsonValue(obj);
    }

    inline JsonValue parseValue(const char*& p) {
        skipWhitespace(p);
        if (*p == '"') return JsonValue(parseString(p));
        if (*p == '{') return parseObject(p);
        if (*p == '[') return parseArray(p);
        if (*p == 't') { p += 4; return JsonValue(true); }
        if (*p == 'f') { p += 5; return JsonValue(false); }
        if (*p == 'n') { p += 4; return JsonValue(); }
        return parseNumber(p);
    }
} // namespace json_detail

/** Parse a JSON string into a JsonValue tree. */
inline JsonValue parseJson(const std::string& text) {
    const char* p = text.c_str();
    return json_detail::parseValue(p);
}

/** Parse JSON from raw bytes (e.g. embedded asset). */
inline JsonValue parseJson(const char* data, size_t length) {
    std::string text(data, length);
    return parseJson(text);
}

// ── JsonBuilder (streaming serialiser for telemetry) ────────────────────────

class JsonBuilder {
    std::ostringstream m_ss;
    bool m_first = true;

public:
    JsonBuilder& beginObject() { m_ss << '{'; m_first = true; return *this; }
    JsonBuilder& endObject()   { m_ss << '}'; return *this; }
    JsonBuilder& beginArray()  { m_ss << '['; m_first = true; return *this; }
    JsonBuilder& endArray()    { m_ss << ']'; return *this; }

    JsonBuilder& key(const std::string& k) {
        if (!m_first) m_ss << ',';
        m_ss << '"' << k << "\":";
        m_first = false;
        return *this;
    }
    JsonBuilder& value(float v)              { m_ss << v; return *this; }
    JsonBuilder& value(int v)                { m_ss << v; return *this; }
    JsonBuilder& value(const std::string& v) { m_ss << '"' << v << '"'; return *this; }
    JsonBuilder& rawValue(float v) {
        if (!m_first) m_ss << ',';
        m_ss << v;
        m_first = false;
        return *this;
    }

    std::string str() const { return m_ss.str(); }
    void reset() { m_ss.str(""); m_ss.clear(); m_first = true; }
};
