// kayros (M13.2) — graceful OOM self-rejection guard. NOT upstream goc code.
//
// Process-global RSS watermark: the prover polls it at the same loop heads as
// the M5.2 deadline and unwinds with MemoryLimitReached instead of being
// OS-OOM-killed with no verdict (the Vu2020 full-horizon TDVRP pathology:
// trivial-TW pruning is weak, label accumulation exceeds any node's RAM).
// Analogous to the poison-and-continue and time-limit guards: the outcome is
// an honest "resource limit, no certificate" result with valid bounds.
//
// The guard observes only: it reads /proc/self/statm and touches no numeric
// state, so solver arithmetic is bit-identical whether or not a (non-tripped)
// limit is set. Polling is throttled to one statm read per kPollStride calls;
// once tripped the state is sticky until the next SetLimitBytes, so every
// loop level unwinds promptly even though freed labels may lower the RSS.

#ifndef GOC_BASE_MEMORY_MONITOR_H
#define GOC_BASE_MEMORY_MONITOR_H

#include <atomic>
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
	// limit <= 0 disables the guard. Resets the sticky tripped state and the
	// poll countdown (call once at solve start).
	static void SetLimitBytes(int64_t limit)
	{
		limit_bytes_.store(limit, std::memory_order_relaxed);
		exceeded_.store(false, std::memory_order_relaxed);
		countdown_.store(0, std::memory_order_relaxed);
	}

	static int64_t LimitBytes() { return limit_bytes_.load(std::memory_order_relaxed); }

	// Throttled watermark check: reads the RSS at most once per kPollStride
	// calls, sticky once tripped. Safe to call in hot loops.
	static bool Exceeded()
	{
		const int64_t limit = limit_bytes_.load(std::memory_order_relaxed);
		if (limit <= 0) return false;
		if (exceeded_.load(std::memory_order_relaxed)) return true;
		if (countdown_.fetch_sub(1, std::memory_order_relaxed) > 0) return false;
		countdown_.store(kPollStride, std::memory_order_relaxed);
		if (CurrentRSSBytes() >= limit)
		{
			exceeded_.store(true, std::memory_order_relaxed);
			return true;
		}
		return false;
	}

	// Current resident set size in bytes (0 when unavailable, e.g. non-Linux:
	// the guard then never trips — same as disabled, never a false verdict).
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
	// observability only — never used in control flow.
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
	// stride 64 keeps the overshoot in the low hundreds of MB — dust against
	// the headroom the auto limit leaves). One statm read costs ~2 us; label
	// iterations are far heavier, so the amortized cost is negligible.
	static constexpr int kPollStride = 64;
	inline static std::atomic<int64_t> limit_bytes_{0};
	inline static std::atomic<bool> exceeded_{false};
	inline static std::atomic<int> countdown_{0};
};
} // namespace goc

#endif // GOC_BASE_MEMORY_MONITOR_H
