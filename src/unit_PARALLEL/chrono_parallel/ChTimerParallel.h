#ifndef CHTIMERPARALLEL_H
#define CHTIMERPARALLEL_H

#include "core/ChTimer.h"

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/math/ChThrustLinearAlgebra.h"

namespace chrono {

struct TimerData {
	double flop;
	double memory_ops;

	ChTimer<double> timer;
	double time;
	double flop_rate;
	double bandwidth;
	int runs;
	bool compute_stats;
	TimerData() {
		flop = memory_ops = 0;
		time = flop_rate = bandwidth = 0;
		runs = 0;
		compute_stats = false;
	}

	void Reset() {
		flop = memory_ops = 0;
		time = flop_rate = bandwidth = 0;
		runs = 0;
	}

	double GetSec() {
		return timer();
	}

	double GetMsec() {

		return timer() / 1000.0;
	}
	void Compute() {
		if (compute_stats) {
			flop_rate = flop * runs / time / 1.0e9;
			bandwidth = (memory_ops * runs / time) / 1024.0 / 1024.0 / 1024.0;
		}
	}

	void start() {
		runs++;
		timer.start();
	}

	void stop() {
		timer.stop();
		time += GetSec();
	}
};

class CH_PARALLEL_API ChTimerParallel {
public:
	ChTimerParallel() {
		total_timers = 0;
		total_time = average_flops = average_bandwidth = 0;
	}
	~ChTimerParallel() {
	}

	void AddTimer(string name) {
		TimerData temp;
		timer_list[name] = temp;
		total_timers++;
	}

	void Reset() {
		for (map<string, TimerData>::iterator it = timer_list.begin(); it != timer_list.end(); it++) {
			it->second.Reset();
		}

	}
	void SetFlop(string name, double f) {

		timer_list[name].flop = f;
		timer_list[name].compute_stats = true;
	}

	void SetMemory(string name, double m) {

		timer_list[name].memory_ops = m;
		timer_list[name].compute_stats = true;
	}

	void start(string name) {
		timer_list[name].start();
	}

	void stop(string name) {
		timer_list[name].stop();

	}
	double GetTime(string name) {
		return timer_list[name].time;
	}
	void PrintReport() {
		total_time = average_flops = average_bandwidth = 0;
		cout << "Timer Report:" << endl;
		cout << "------------" << endl;
		for (map<string, TimerData>::iterator it = timer_list.begin(); it != timer_list.end(); it++) {
			it->second.Compute();
			cout << "Name:\t" << it->first << "\t" << it->second.time;
			if (it->second.compute_stats) {
				cout << "\t" << it->second.flop_rate << "\t" << it->second.bandwidth;
				average_flops += it->second.flop_rate;
				average_bandwidth += it->second.bandwidth;
			}
			cout << endl;
			total_time += it->second.time;

		}
		cout << "------------" << endl;
		//cout << total_time << " " << average_flops / total_timers << " " << average_bandwidth / total_timers << endl;
	}

	double total_time;
	double average_flops;
	double average_bandwidth;
	int total_timers;
	std::map<string, TimerData> timer_list;
	map<string, TimerData>::iterator it;
};

}

#endif
