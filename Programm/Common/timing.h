
#ifndef timing_H
#define timing_H

#define USE_TIMING

#ifdef USE_TIMING
#ifdef _MSC_VER
#include <Windows.h>
#include <iostream>
#include <stack>
#include <hash_map>
#include "Common/Config.h"

#define START_TIMING(timerName) \
	USE_TIMESTEP_TIMING(startTiming(timerName));

#define STOP_TIMING \
	USE_TIMESTEP_TIMING(stopTiming(false));

#define STOP_TIMING_PRINT \
	USE_TIMESTEP_TIMING(stopTiming(true));

#define STOP_TIMING_AVG \
	{ \
	static int timing_timerId = -1; \
	USE_TIMESTEP_TIMING(stopTiming(false, timing_timerId)); \
	}

#define STOP_TIMING_AVG_PRINT \
	{ \
	static int timing_timerId = -1; \
	USE_TIMESTEP_TIMING(stopTiming(true, timing_timerId)); \
	}


struct TimingHelper
{
	LARGE_INTEGER start;
	std::string name;
};

struct AverageTime
{
	double totalTime;
	unsigned int counter;
	std::string name;
};

class Timing
{
private:
	static int m_currentId;

public:
	static bool m_dontPrintTimes;
	static LARGE_INTEGER freq;
	static std::stack<TimingHelper> m_timingStack;
	static stdext::hash_map<int, AverageTime> m_averageTimes;

	static int getId() { return m_currentId++; }
};

FORCE_INLINE void startTiming(const std::string& name = std::string(""))
{
	QueryPerformanceFrequency(&Timing::freq);
	TimingHelper h;
	QueryPerformanceCounter(&h.start);
	h.name = name;
	Timing::m_timingStack.push(h);
}

FORCE_INLINE double stopTiming(bool print=true)
{
	if(!Timing::m_timingStack.empty())
	{
		LARGE_INTEGER  stop;
		QueryPerformanceCounter(&stop);
		TimingHelper h = Timing::m_timingStack.top();
		Timing::m_timingStack.pop();
		const double t = (1000.0 * (stop.QuadPart - h.start.QuadPart)/(double)Timing::freq.QuadPart);
		if (print && !Timing::m_dontPrintTimes)
			std::cout << "time " << h.name.c_str() << ": " << t << " ms\n" << std::flush;
		return t;
	}
	return 0;
}

FORCE_INLINE double stopTiming(bool print, int &id)
{
	if (id == -1)
		id = Timing::getId();
	if(!Timing::m_timingStack.empty())
	{
		LARGE_INTEGER  stop;
		QueryPerformanceCounter(&stop);
		TimingHelper h = Timing::m_timingStack.top();
		Timing::m_timingStack.pop();
		const double t = (1000.0 * (stop.QuadPart - h.start.QuadPart)/(double)Timing::freq.QuadPart);
		if (print && !Timing::m_dontPrintTimes)
			std::cout << "time " << h.name.c_str() << ": " << t << " ms\n" << std::flush;

		if (id >= 0)
		{
			stdext::hash_map <int, AverageTime>::iterator iter;
			iter = Timing::m_averageTimes.find(id);
			if (iter != Timing::m_averageTimes.end())
			{
				Timing::m_averageTimes[id].totalTime += t;
				Timing::m_averageTimes[id].counter++;
			}
			else
			{
				AverageTime at;
				at.counter = 1;
				at.totalTime = t;
				at.name = h.name;
				Timing::m_averageTimes[id] = at;
			}
		}
		return t;
	}
	return 0;
}


FORCE_INLINE void printAverageTimes()
{
	stdext::hash_map <int, AverageTime>::iterator iter;
	for (iter = Timing::m_averageTimes.begin(); iter != Timing::m_averageTimes.end(); iter++)
	{
		AverageTime &at = iter->second;
		const double avgTime = at.totalTime / at.counter;
		std::cout << "Average time " << at.name.c_str() << ": " << avgTime << " ms\n" << std::flush;
	}
}

FORCE_INLINE void printTimeSums()
{
	stdext::hash_map <int, AverageTime>::iterator iter;
	for (iter = Timing::m_averageTimes.begin(); iter != Timing::m_averageTimes.end(); iter++)
	{
		AverageTime &at = iter->second;
		const double timeSum = at.totalTime;
		std::cout << "Time sum " << at.name.c_str() << ": " << timeSum << " ms\n" << std::flush;
	}
}


#else

#include <iostream>
#include <stack>
//#include <CGAL/Timer.h>

struct TimingHelper
{
	//CGAL::Timer* timer;
	std::string name;
};

static std::stack<TimingHelper> timingStack;

inline void startTiming(const std::string& name = std::string(""))
{
	/*TimingHelper h;
	h.timer = new CGAL::Timer();
	h.name = name;
	timingStack.push(h);
	h.timer->start();*/
}

inline double stopTiming()
{
	if(!timingStack.empty())
	{
		/*TimingHelper h = timingStack.top();
		timingStack.pop();
		h.timer->stop();
		std::cout << "time " << h.name.c_str() << ": " << h.timer->time() * 1000.0 << " ms\n";
		delete h.timer;*/
	}
}

#endif

#else

#include <iostream>

inline void startTiming(const std::string& name = std::string(""))
{
}

inline void stopTiming()
{
}

#endif

#endif