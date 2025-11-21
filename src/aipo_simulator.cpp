// AIPAO simulator: small header comment
// src/aipo_simulator.cpp
// Patched AI-powered Performance Analyzer for OS Processes
// Adds: stable regression, clamped forecasts, robust analysis loop, CSV export
// Compile: g++ -std=c++17 src/aipo_simulator.cpp -O2 -o aipo_sim.exe
// Run: .\aipo_sim.exe traces\sample_burst.txt   (Windows PowerShell)

#include <bits/stdc++.h>
using namespace std;

struct Process {
    int pid;
    double arrival; // ms
    double burst;   // ms total work
    double remaining;
    double mem_kb;  // simulated memory footprint
    double io_weight; // 0..1
    double start_time, finish_time;
    double cpu_consumed; // ms
    Process(int id=0,double a=0,double b=0,double m=0,double io=0)
      :pid(id),arrival(a),burst(b),remaining(b),mem_kb(m),io_weight(io),
       start_time(-1),finish_time(-1),cpu_consumed(0){}
};

struct SeriesPoint { double time; double value; };

struct Analyzer {
    // moving average over window_ms of last points
    static double moving_avg(const vector<SeriesPoint>& s, double window_ms) {
        if(s.empty()) return 0;
        double now = s.back().time;
        double sum=0; int cnt=0;
        for(int i=(int)s.size()-1;i>=0;--i){
            if(now - s[i].time <= window_ms){ sum+=s[i].value; cnt++; } else break;
        }
        return cnt? sum/cnt : 0.0;
    }
    // stable linear regression: uses time-offset (small x), returns (slope, intercept)
    static pair<double,double> linear_regression_offset(const vector<SeriesPoint>& s, int last_n){
        int n = min((int)s.size(), last_n);
        const int MIN_POINTS_FOR_REG = 5;
        if(n < MIN_POINTS_FOR_REG) {
            double y = s.empty()?0.0:s.back().value;
            return {0.0, y};
        }
        int start = (int)s.size() - n;
        double t0 = s[start].time;
        double sx=0, sy=0, sxx=0, sxy=0;
        for(int i=start;i<s.size();++i){
            double x = s[i].time - t0; // offset time
            double y = s[i].value;
            sx += x; sy += y; sxx += x*x; sxy += x*y;
        }
        double denom = n * sxx - sx * sx;
        if(fabs(denom) < 1e-9) {
            double y = sy / n;
            return {0.0, y};
        }
        double m = (n * sxy - sx * sy) / denom;
        double c = (sy - m * sx) / n;
        // intercept is c but with respect to offset; convert to absolute intercept at time 0:
        double abs_intercept = c - m * (-t0); // c + m * t0
        abs_intercept = c + m * ( - (t0 - t0) ); // simplified; we'll return slope & c (relative)
        // We will return (slope, current_estimate) where current_estimate = predicted value at last time
        double last_x = s.back().time - t0;
        double pred_last = m * last_x + c;
        return {m, pred_last};
    }
};

struct Simulator {
    vector<Process> procs;
    double current_time = 0.0; // ms
    double quantum = 10.0; // ms
    vector<SeriesPoint> cpu_util_ts; // time->util (0..100)
    vector<SeriesPoint> mem_usage_ts; // time->mem_kb_total
    double max_observed_mem = 0.0;

    // CSV writer
    ofstream csv;

    void open_csv(const string &path="analysis.csv"){
        csv.open(path);
        csv << "time_ms,avg_cpu_util,mem_kb,slope_kb_per_ms,forecast_kb,"
               "top1_pid,top1_cpu_ms,top2_pid,top2_cpu_ms,top3_pid,top3_cpu_ms,hotspots\n";
    }
    void close_csv(){ if(csv.is_open()) csv.close(); }

    void load(const vector<tuple<double,double,double,double>>& jobs){
        procs.clear(); int id=1;
        for(auto &t: jobs) procs.emplace_back(id++, get<0>(t), get<1>(t), get<2>(t), get<3>(t));
        current_time = 0.0;
        cpu_util_ts.clear();
        mem_usage_ts.clear();
        max_observed_mem = 0.0;
    }

    int pick_next(){
        int idx=-1; double best=1e18;
        for(int i=0;i<procs.size();++i)
            if(procs[i].arrival<=current_time && procs[i].remaining>1e-9)
                if(procs[i].remaining < best){ best = procs[i].remaining; idx=i; }
        return idx;
    }

    bool all_done(){ for(auto &p: procs) if(p.remaining>1e-9) return false; return true; }

    void step(){
        int idx = pick_next();
        if(idx<0){
            double tnext = 1e18;
            for(auto &p: procs) if(p.arrival>current_time) tnext=min(tnext,p.arrival);
            if(tnext==1e18) return; // all done
            // jump to next arrival (idle)
            current_time = tnext;
            cpu_util_ts.push_back({current_time,0});
            mem_usage_ts.push_back({current_time, total_mem()});
            max_observed_mem = max(max_observed_mem, total_mem());
            return;
        }
        Process &pr = procs[idx];
        if(pr.start_time<0) pr.start_time = current_time;
        double run = min(quantum, pr.remaining / max(1.0 - pr.io_weight, 1e-9)); // ensure some progress
        if(run <= 0) run = quantum;
        // CPU effective work is reduced by io_weight
        double cpu_run = run * (1.0 - pr.io_weight);
        // guard: cpu_run could be zero for io_weight ~1. Still we must advance time by 'run'
        pr.remaining -= cpu_run;
        if(pr.remaining < 0) pr.remaining = 0;
        pr.cpu_consumed += cpu_run;
        current_time += run;
        double util = instant_cpu_util();
        cpu_util_ts.push_back({current_time, util});
        mem_usage_ts.push_back({current_time, total_mem()});
        max_observed_mem = max(max_observed_mem, total_mem());
        if(pr.remaining <= 1e-9) pr.finish_time = current_time;
    }

    double total_mem(){
        double s=0; for(auto &p: procs) if(p.arrival<=current_time && p.remaining>1e-9) s+=p.mem_kb; return s;
    }
    double instant_cpu_util(){
        double busy=0;
        for(auto &p: procs) if(p.arrival<=current_time && p.remaining>1e-9) busy += max(0.0, 1.0 - p.io_weight);
        double max_possible = max(1.0, (double)procs.size());
        double util = min(100.0, (busy/max_possible)*100.0);
        return util;
    }

    void run_and_analyze(){
        open_csv();
        double analysis_interval = 100.0; double next_analysis = analysis_interval;
        const double EPS = 1e-6;
        // initial record
        cpu_util_ts.push_back({current_time, 0.0});
        mem_usage_ts.push_back({current_time, total_mem()});
        while(!all_done()){
            double prev_time = current_time;
            step();
            if(current_time <= prev_time + EPS){
                // ensure progress: jump to next arrival or add tiny epsilon
                double tnext = 1e18;
                for(auto &p: procs) if(p.arrival>current_time) tnext=min(tnext,p.arrival);
                if(tnext==1e18) break;
                current_time = max(current_time + 1.0, tnext); // advance by 1 ms
            }
            // robust analysis loop (handles multiple missed intervals)
            while(current_time >= next_analysis){
                analyze_and_report(next_analysis);
                next_analysis += analysis_interval;
            }
        }
        // final analysis (at end time)
        analyze_and_report(current_time);
        close_csv();
    }

    void analyze_and_report(double at_time){
        cout << "\n--- Analysis at t=" << (int)round(at_time) << " ms ---\n";
        // top CPU consumers
        vector<pair<double,int>> cpu_consumers;
        for(int i=0;i<procs.size();++i) cpu_consumers.push_back({procs[i].cpu_consumed, i});
        sort(cpu_consumers.rbegin(), cpu_consumers.rend());
        cout << "Top CPU consumers:\n";
        for(int k=0;k<min(3,(int)cpu_consumers.size());++k){
            auto &pr = procs[cpu_consumers[k].second];
            cout << " P"<<pr.pid<<" cpu_ms="<< (int)round(pr.cpu_consumed) <<" mem="<< (int)pr.mem_kb <<" io="<<pr.io_weight<<"\n";
        }
        double avg_util = Analyzer::moving_avg(cpu_util_ts, 200.0);
        cout << "Avg CPU util (recent 200ms) = "<< fixed << setprecision(2) << avg_util <<"%\n";

        // regression (slope estimate) with offset and stability
        auto reg = Analyzer::linear_regression_offset(mem_usage_ts, 10);
        double slope = reg.first; // kb per ms approx
        double last_mem = mem_usage_ts.empty()?0.0:mem_usage_ts.back().value;
        double forecast = last_mem + slope * 500.0; // 500ms ahead
        // clamp forecast
        double cap = max( (double)0.0, 2.0 * max_observed_mem );
        if(cap < 1.0) cap = max( (double)100.0, last_mem * 2.0 );
        if(forecast < 0.0) forecast = 0.0;
        if(forecast > cap) forecast = cap;

        cout << "Memory slope = " << setprecision(4) << slope << " kb/ms. Forecast in 500ms = " << (long long)round(forecast) << " kb\n";
        if(forecast > 1024.0 * 1024.0) cout << "Warning: projected memory > 1GB, suggest reduce working set or enable swap.\n";

        int hotspots = 0;
        for(auto &p: procs){
            if(p.cpu_consumed > 100 && p.remaining > 50){
                cout<<"Hotspot detected: P"<<p.pid<<" (cpu_ms="<<(int)round(p.cpu_consumed)<<", rem="<<(int)round(p.remaining)<<"ms)\n";
                cout<<"Suggestion: consider lowering priority or parallelizing workload.\n";
                hotspots++;
            }
        }
        // classification
        for(auto &p: procs){
            if(p.cpu_consumed > 0){
                double cpu_frac = p.cpu_consumed / max(1.0, (double)p.burst);
                if(cpu_frac>0.7) cout<<"P"<<p.pid<<" classified: CPU-bound\n";
                else if(p.io_weight>0.6) cout<<"P"<<p.pid<<" classified: IO-bound\n";
                else cout<<"P"<<p.pid<<" classified: Mixed\n";
            }
        }
        cout << "Gantt snapshot (pid:remaining_ms): ";
        for(auto &p: procs) if(p.arrival<=current_time && p.remaining>1e-9) cout<<"[P"<<p.pid<<":"<<(int)round(p.remaining)<<"ms] ";
        cout << "\n";

        // write CSV row: time,avg_util,mem,slope,forecast,top3 pids+cpu,hotspots
        int t1_pid=-1; long long t1_cpu=0, t2_cpu=0, t3_cpu=0; int t2_pid=-1, t3_pid=-1;
        if(cpu_consumers.size()>0){ t1_pid = procs[cpu_consumers[0].second].pid; t1_cpu = (long long)round(cpu_consumers[0].first); }
        if(cpu_consumers.size()>1){ t2_pid = procs[cpu_consumers[1].second].pid; t2_cpu = (long long)round(cpu_consumers[1].first); }
        if(cpu_consumers.size()>2){ t3_pid = procs[cpu_consumers[2].second].pid; t3_cpu = (long long)round(cpu_consumers[2].first); }
        if(csv.is_open()){
            csv << (long long)round(at_time) << "," << fixed << setprecision(3) << avg_util << "," << (long long)round(last_mem)
                << "," << slope << "," << (long long)round(forecast) << ","
                << t1_pid << "," << t1_cpu << "," << t2_pid << "," << t2_cpu << "," << t3_pid << "," << t3_cpu << "," << hotspots << "\n";
        }
    }
};

vector<tuple<double,double,double,double>> sample_jobs(){
    return {
        {0, 200, 20000, 0.1},
        {20, 80, 10000, 0.7},
        {40, 150, 50000, 0.2},
        {100, 400, 120000, 0.05},
        {250, 60, 8000, 0.8},
    };
}

int main(int argc, char** argv){
    ios::sync_with_stdio(false); cin.tie(nullptr);
    vector<tuple<double,double,double,double>> jobs;
    if(argc>1){
        // expect path relative to project root, e.g. traces\sample_burst.txt
        ifstream ifs(argv[1]);
        if(!ifs){ cerr<<"Cannot open "<<argv[1]<<"\n"; return 1; }
        double a,b,m,io;
        while(ifs>>a>>b>>m>>io) jobs.emplace_back(a,b,m,io);
    } else {
        jobs = sample_jobs();
        cout<<"No trace file given — using sample jobset.\n";
    }
    Simulator sim; sim.load(jobs); sim.run_and_analyze();
    cout<<"\nSimulation finished. CSV saved to analysis.csv (in current folder).\n";
    return 0;
}
