// eadvfs_simulator.cpp
// Energy-Aware DVFS Scheduler Simulator (EADVFS)
// Compile: g++ -std=c++17 eadvfs_simulator.cpp -O2 -o eadvfs_sim
// Run: ./eadvfs_sim <input_file>   (or run without args to use sample generator)

#include <bits/stdc++.h>
using namespace std;

struct Process {
    int pid;
    double arrival;    // ms
    double burst;      // ms (total)
    double remaining;  // ms
    double start_time; // first start (ms), -1 if not started
    double finish_time; // ms, -1 if not finished
    Process(int id=0,double a=0,double b=0):pid(id),arrival(a),burst(b),remaining(b),
        start_time(-1),finish_time(-1){}
};

struct FreqLevel {
    double ghz;    // relative frequency number
    double power;  // power in Watts at this frequency (includes static)
    string name;
    FreqLevel(double g=1.0,double p=1.0,string n="f"):ghz(g),power(p),name(n){}
};

// Simple PowerModel
struct PowerModel {
    vector<FreqLevel> freqs;
    double idle_power; // Watts (deep idle)
    PowerModel() {
        // Example levels: these numbers are illustrative
        freqs.push_back(FreqLevel(1.0, 1.5, "1.0GHz")); // lower power
        freqs.push_back(FreqLevel(1.5, 2.6, "1.5GHz"));
        freqs.push_back(FreqLevel(2.0, 4.5, "2.0GHz")); // higher power
        idle_power = 0.2;
    }
    // choose index by heuristic externally
};

// Scheduler implementing EADVFS heuristic
struct Scheduler {
    PowerModel &pm;
    double short_threshold; // ms: remaining <= threshold considered short
    double util_threshold;  // utilization threshold to push high freq
    Scheduler(PowerModel &p):pm(p) {
        short_threshold = 30.0; // 30 ms
        util_threshold = 0.6;
    }

    // heuristic to pick frequency index based on ready queue workload and predicted util
    int pick_frequency_index(const vector<Process*> &ready, double lookahead_window_ms) {
        if(ready.empty()) return -1;
        double sum_rem = 0;
        int short_count = 0;
        for(auto pr: ready) {
            sum_rem += pr->remaining;
            if(pr->remaining <= short_threshold) short_count++;
        }
        double avg_rem = sum_rem / ready.size();
        double short_frac = (double)short_count / ready.size();

        // predicted utilization in next window approx = (sum_rem / lookahead_window)
        double util_pred = min(1.0, sum_rem / max(1.0, lookahead_window_ms));

        // Heuristic:
        // - If many short jobs (short_frac high) -> pick high freq
        // - If predicted util high -> pick high freq to reduce queue buildup
        // - If avg_rem is high and util moderate -> pick medium/low freq
        if(short_frac > 0.6 || util_pred > util_threshold) {
            return (int)pm.freqs.size()-1; // highest freq
        } else if(avg_rem > 200.0) {
            return 0; // lowest freq for long jobs
        } else {
            return 1; // medium
        }
    }

    // pick index of next process (SRTF: smallest remaining)
    int pick_next_process_index(const vector<Process*> &ready) {
        if(ready.empty()) return -1;
        int idx = 0;
        double best = ready[0]->remaining;
        for(size_t i=1;i<ready.size();++i) {
            if(ready[i]->remaining < best) { best = ready[i]->remaining; idx = i; }
        }
        return idx;
    }
};

// Simulator
struct Simulator {
    vector<Process> procs;
    PowerModel pm;
    Scheduler scheduler;
    double current_time; // ms
    double energy; // Joules
    double busy_time; // ms
    vector<pair<int,double>> gantt; // (pid, duration)

    Simulator():pm(),scheduler(pm) {
        current_time = 0.0;
        energy = 0.0;
        busy_time = 0.0;
    }

    void load_processes(const vector<pair<double,double>>& list) {
        procs.clear();
        int id=1;
        for(auto &p: list) {
            procs.emplace_back(id++, p.first, p.second);
        }
    }

    void simulate_eadvfs(double sim_end_ms = 10000.0) {
        energy = 0.0;
        busy_time = 0.0;
        current_time = 0.0;
        for(auto &pr: procs) { pr.remaining = pr.burst; pr.start_time=-1; pr.finish_time=-1; }

        // event-driven: advance to next arrival or until completion
        while(true) {
            // build ready queue
            vector<Process*> ready;
            Process* next_arrival = nullptr;
            double next_arrival_time = 1e18;
            for(auto &p: procs) {
                if(p.arrival <= current_time && p.remaining > 1e-9) ready.push_back(&p);
                if(p.arrival > current_time && p.arrival < next_arrival_time) {
                    next_arrival_time = p.arrival;
                    next_arrival = &p;
                }
            }
            if(ready.empty()) {
                // nothing to run now
                // jump to next arrival or finish
                if(next_arrival == nullptr) break; // all done
                // go to idle until next_arrival_time
                double idle_for = next_arrival_time - current_time;
                // consume idle energy
                energy += pm.idle_power * (idle_for/1000.0); // convert ms to s
                current_time = next_arrival_time;
                continue;
            }

            // Decide frequency
            int fi = scheduler.pick_frequency_index(ready, 200.0); // lookahead 200ms
            if(fi < 0) fi = 0;
            FreqLevel fl = pm.freqs[fi];

            // pick process to run (SRTF)
            int pi = scheduler.pick_next_process_index(ready);
            Process* p = ready[pi];
            // Determine time slice: run either until preemption by arrival or completion or step (e.g. 10 ms)
            double next_event_time = 1e18;
            // next arrival can preempt
            for(auto &pr: procs) if(pr.arrival > current_time) next_event_time = min(next_event_time, pr.arrival);
            // compute completion time at chosen frequency: runtime scales inversely with freq relative to baseline 1.0GHz
            double effective_speed = fl.ghz; // relative
            double time_to_finish = p->remaining / effective_speed;
            double completion_time = current_time + time_to_finish;
            double run_until = completion_time;
            if(next_event_time < run_until) run_until = next_event_time; // preempt by new arrival
            // run in small quanta to make Gantt readable if very long (cap quantum)
            double quantum = 50.0; // ms
            if(run_until - current_time > quantum) run_until = current_time + quantum;

            double run_time = run_until - current_time; // ms
            if(run_time <= 0) { current_time = run_until; continue; }

            // log start time
            if(p->start_time < 0) p->start_time = current_time;
            // progress
            double work_done = run_time * effective_speed; // ms of work done at 1.0GHz baseline
            p->remaining = max(0.0, p->remaining - work_done);
            // energy consumed: fl.power (Watts) * time_s
            energy += fl.power * (run_time/1000.0);
            busy_time += run_time;
            // Gantt: merge same pid if adjacent
            if(!gantt.empty() && gantt.back().first == p->pid) gantt.back().second += run_time;
            else gantt.push_back({p->pid, run_time});
            current_time += run_time;

            if(p->remaining <= 1e-9) {
                p->finish_time = current_time;
            }
            // continue loop
            // check if all done
            bool any_left=false;
            for(auto &pr: procs) if(pr.remaining > 1e-9) { any_left = true; break; }
            if(!any_left) break;
            if(current_time > sim_end_ms) break;
        }
    }

    void print_stats_and_gantt() {
        cout << "===== EADVFS Simulation Results =====\n";
        double total_wait = 0.0, total_turn = 0.0;
        int n = procs.size();
        for(auto &p: procs) {
            double tat = (p.finish_time - p.arrival);
            double wait = tat - p.burst;
            total_turn += tat;
            total_wait += wait;
        }
        cout << "Processes: " << n << "\n";
        cout << fixed << setprecision(3);
        cout << "Avg Turnaround (ms): " << (total_turn/n) << "\n";
        cout << "Avg Waiting (ms): " << (total_wait/n) << "\n";
        double makespan = 0;
        for(auto &p: procs) makespan = max(makespan, p.finish_time);
        cout << "Makespan (ms): " << makespan << "\n";
        cout << "Total Energy (J): " << energy << "\n";
        cout << "CPU Utilization (%): " << (busy_time / max(1.0, makespan) * 100.0) << "\n\n";

        // Gantt chart (simple)
        cout << "Gantt chart (pid:duration_ms):\n";
        for(auto &g: gantt) {
            cout << "[P" << g.first << ":" << (int)round(g.second) << "ms] ";
        }
        cout << "\n\nDetailed per-process:\n";
        for(auto &p: procs) {
            cout << "P" << p.pid << " arrival=" << p.arrival << " burst=" << p.burst
                 << " start=" << p.start_time << " finish=" << p.finish_time << "\n";
        }
    }
};

// Helper: read input from file or generate sample
vector<pair<double,double>> sample_input() {
    // sample mixed jobs (arrival (ms), burst(ms))
    return {
        {0, 120},
        {20, 30},
        {40, 50},
        {100, 200},
        {150, 20},
        {300, 400},
        {350, 60},
    };
}

int main(int argc, char** argv) {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    vector<pair<double,double>> jobs;
    if(argc > 1) {
        // read file: each line "arrival_ms burst_ms"
        ifstream ifs(argv[1]);
        if(!ifs) {
            cerr << "Cannot open file " << argv[1] << "\n";
            return 1;
        }
        double a,b;
        while(ifs >> a >> b) jobs.push_back({a,b});
    } else {
        jobs = sample_input();
        cout << "No input file given — using sample jobset.\n";
    }

    Simulator sim;
    sim.load_processes(jobs);
    sim.simulate_eadvfs(100000.0);
    sim.print_stats_and_gantt();
    return 0;
}
