#pragma once

#include <chrono>
#include <iosfwd>
#include <vector>

// This is a placeholder for C++20's ranges::array_view
template <typename T>
class array_view {
	T const* data;
	size_t count;
public:
	array_view(T const* data, size_t count) : data{data}, count{count} {}
	T const* begin() const { return data; }
	T const* end() const { return data + count; }
	size_t size() const { return count; }
};

template<typename T>
struct period_name;

template<typename period_ = std::micro, typename rep_ = size_t>
class stopwatch {
public:
	using period = period_;
	using rep = rep_;

	stopwatch() {
		begin = end = std::chrono::steady_clock::time_point::min();
	}
	void start(void) {
		begin = std::chrono::steady_clock::now();
	}
	void stop(void) {
		end = std::chrono::steady_clock::now();
	}
	rep count(void) const {
		using namespace std::chrono;
		return duration_cast<std::chrono::duration<rep, period>>(end - begin).count();
	}
	friend std::ostream& operator<<(std::ostream &o, stopwatch const& sw) {
		return o << sw.count() << ' ' << period_name<period>::short_name;
	}
	template<typename ExecFun, typename ReportFun, typename ... ReportArgs>
	void time_it(size_t num_iter, ExecFun f, ReportFun report, ReportArgs&&... args) {
		std::vector<rep> times;
		times.reserve(num_iter);
		for(size_t i=0; i<num_iter; i++) {
			this->start();
			f();
			this->stop();
			times.push_back(this->count());
		}
		report(array_view<rep>(times.data(), times.size()), std::forward<ReportArgs>(args)...);
	}
private:
	std::chrono::time_point<std::chrono::steady_clock> begin, end;
};

template<> struct period_name<std::pico> {
	static constexpr char const* name = "picosecond";
	static constexpr char const* short_name = "ps";
};
template<> struct period_name<std::nano> {
	static constexpr char const* name = "nanosecond";
	static constexpr char const* short_name = "ns";
};
template<> struct period_name<std::micro> {
	static constexpr char const* name = "microsecond";
	static constexpr char const* short_name = "us";
};
template<> struct period_name<std::milli> {
	static constexpr char const* name = "millisecond";
	static constexpr char const* short_name = "ms";
};
template<> struct period_name<std::centi> {
	static constexpr char const* name = "centisecond";
	static constexpr char const* short_name = "cs";
};
template<> struct period_name<std::deci> {
	static constexpr char const* name = "decisecond";
	static constexpr char const* short_name = "ds";
};
template<> struct period_name<std::deca> {
	static constexpr char const* name = "decasecond";
	static constexpr char const* short_name = "das";
};
template<> struct period_name<std::hecto> {
	static constexpr char const* name = "hectosecond";
	static constexpr char const* short_name = "hs";
};
template<> struct period_name<std::kilo> {
	static constexpr char const* name = "kilosecond";
	static constexpr char const* short_name = "ks";
};
template<> struct period_name<std::mega> {
	static constexpr char const* name = "megasecond";
	static constexpr char const* short_name = "Ms";
};
template<> struct period_name<std::giga> {
	static constexpr char const* name = "gigasecond";
	static constexpr char const* short_name = "Gs";
};
template<> struct period_name<std::tera> {
	static constexpr char const* name = "terasecond";
	static constexpr char const* short_name = "Ts";
};
template<> struct period_name<std::peta> {
	static constexpr char const* name = "petasecond";
	static constexpr char const* short_name = "Ps";
};
template<> struct period_name<std::exa> {
	static constexpr char const* name = "exasecond";
	static constexpr char const* short_name = "Es";
};
