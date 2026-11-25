// kayros (M13.2): graceful OOM self-rejection guard. NOT upstream goc code.
//
// Per-thread watermark of process RSS: the prover polls it at the same loop heads as
// the M5.2 deadline and unwinds with MemoryLimitReached instead of being
// OS-OOM-killed with no verdict (the Vu2020 full-horizon TDVRP pathology:
// trivial-TW pruning is weak, label accumulation exceeds any node's RAM).
// Analogous to the poison-and-continue and time-limit guards: the outcome is
// an honest "resource limit, no certificate" result with valid bounds.
//
// The guard observes only: it reads /proc/self/statm and touches no numeric
// state, so solver arithmetic is bit-identical whether or not a (non-tripped)
// limit is set. Polling is throttled to one statm read per kPollStride calls;
// once tripped the state is sticky for that solve, so every
// loop level unwinds promptly even though freed labels may lower the RSS.

#ifndef GOC_BASE_MEMORY_MONITOR_H
#define GOC_BASE_MEMORY_MONITOR_H

#include <cstdint>
#include <cstdio>

#ifdef __linux__
#include <unistd.h>
#endif

namespace goc
{
class MemoryMonitor
{
public:
	explicit MemoryMonitor(int64_t limit) : previous_(state_) { state_ = {limit, false, 0}; }
	~MemoryMonitor() { state_ = previous_; }
	MemoryMonitor(const MemoryMonitor&) = delete;
	MemoryMonitor& operator=(const MemoryMonitor&) = delete;

	static int64_t LimitBytes() { return state_.limit; }

	// Throttled watermark check: reads the RSS at most once per kPollStride
	// calls, sticky once tripped. Safe to call in hot loops.
	static bool Exceeded()
	{
		const int64_t limit = state_.limit;
		if (limit <= 0) return false;
		if (state_.exceeded) return true;
		if (state_.countdown-- > 0) return false;
		state_.countdown = kPollStride;
		if (CurrentRSSBytes() >= limit)
		{
			state_.exceeded = true;
			return true;
		}
		return false;
	}

	// Current resident set size in bytes (0 when unavailable, e.g. non-Linux:
	// the guard then never trips; same as disabled, never a false verdict).
	static int64_t CurrentRSSBytes()
	{
#ifdef __linux__
		FILE* f = std::fopen("/proc/self/statm", "r");
		if (!f) return 0;
		long long size_pages = 0, resident_pages = 0;
		const int matched = std::fscanf(f, "%lld %lld", &size_pages, &resident_pages);
		std::fclose(f);
		if (matched != 2) return 0;
		return (int64_t)resident_pages * (int64_t)sysconf(_SC_PAGESIZE);
#else
		return 0;
#endif
	}

	// Peak resident set size in bytes (VmHWM; 0 when unavailable). For result
	// observability only, never used in control flow.
	static int64_t PeakRSSBytes()
	{
#ifdef __linux__
		FILE* f = std::fopen("/proc/self/status", "r");
		if (!f) return 0;
		char line[256];
		int64_t peak_kb = 0;
		while (std::fgets(line, sizeof(line), f))
		{
			long long kb = 0;
			if (std::sscanf(line, "VmHWM: %lld kB", &kb) == 1) { peak_kb = kb; break; }
		}
		std::fclose(f);
		return peak_kb * 1024;
#else
		return 0;
#endif
	}

private:
	// Full-horizon labels allocate MBs each, so the stride bounds the RSS
	// overshoot past the watermark (measured ~3 MB/label on Vu2020 n=59:
	// stride 64 keeps the overshoot in the low hundreds of MB, dust against
	// the headroom the auto limit leaves). One statm read costs ~2 us; label
	// iterations are far heavier, so the amortized cost is negligible.
	static constexpr int kPollStride = 64;
	struct State {
		int64_t limit;
		bool exceeded;
		int countdown;
	};
	inline static thread_local State state_{};
	State previous_;
};
} // namespace goc

#endif // GOC_BASE_MEMORY_MONITOR_H
