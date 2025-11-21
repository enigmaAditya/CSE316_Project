// src/aipo_simulator.cpp
// AI-powered Performance Analyzer for OS Processes (Phase-1 prototype)
// Compile: g++ -std=c++17 src/aipo_simulator.cpp -O2 -o aipo_sim
// Run: ./aipo_sim <trace_file>  (or run without args to use sample generator)

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
    static double moving_avg(const vector<SeriesPoint>& s, double window_ms) {
        if(s.empty()) return 0;
        double now = s.back().time;
        double sum=0; int cnt=0;
        for(int i=(int)s.size()-1;i>=0;--i){
            if(now - s[i].time <= window_ms){ sum+=s[i].value; cnt++; } else break;
        }
        return cnt? sum/cnt : 0.0;
    }
    static pair<double,double> linear_regression(const vector<SeriesPoint>& s, int last_n){
        int n = min((int)s.size(), last_n);
        if(n<2) return {0, s.empty()?0:s.back().value};
        double sx=0, sy=0, sxx=0, sxy=0;
        for(int i=s.size()-n;i<s.size();++i){ double x = s[i].time; double y = s[i].value; sx+=x; sy+=y; sxx+=x*x; sxy+=x*y; }
        double denom = n*sxx - sx*sx; if(fabs(denom) < 1e-9) return {0, sy/n};
        double m = (n*sxy - sx*sy)/denom; double c = (sy - m*sx)/n; return {m,c};
    }
};

struct Simulator {
    vector<Process> procs;
    double current_time = 0.0; // ms
    double quantum = 10.0; // ms
    vector<SeriesPoint> cpu_util_ts; // time->util (0..100)
    vector<SeriesPoint> mem_usage_ts; // time->mem_kb_total

    void load(const vector<tuple<double,double,double,double>>& jobs){
        procs.clear(); int id=1;
        for(auto &t: jobs) procs.emplace_back(id++, get<0>(t), get<1>(t), get<2>(t), get<3>(t));
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
            if(tnext==1e18) return;
            current_time = tnext;
            cpu_util_ts.push_back({current_time,0}); mem_usage_ts.push_back({current_time, total_mem()});
            return;
        }
        Process &pr = procs[idx];
        if(pr.start_time<0) pr.start_time = current_time;
        double run = min(quantum, pr.remaining);
        double cpu_run = run * (1.0 - pr.io_weight);
        pr.remaining -= cpu_run; pr.cpu_consumed += cpu_run; current_time += run;
        double util = instant_cpu_util();
        cpu_util_ts.push_back({current_time, util}); mem_usage_ts.push_back({current_time, total_mem()});
        if(pr.remaining <= 1e-9) pr.finish_time = current_time;
    }

    double total_mem(){
        double s=0; for(auto &p: procs) if(p.arrival<=current_time && p.remaining>1e-9) s+=p.mem_kb; return s;
    }
    double instant_cpu_util(){
        double busy=0;
        for(auto &p: procs) if(p.arrival<=current_time && p.remaining>1e-9) busy += 1.0 - p.io_weight;
        double max_possible = max(1.0, (double)procs.size());
        double util = min(100.0, (busy/max_possible)*100.0);
        return util;
    }

    void run_and_analyze(){
        double analysis_interval = 100.0; double next_analysis = analysis_interval;
        while(!all_done()){
            step();
            if(current_time >= next_analysis){ analyze_and_report(next_analysis); next_analysis += analysis_interval; }
        }
        analyze_and_report(current_time);
    }

    void analyze_and_report(double at_time){
        cout << "\n--- Analysis at t=" << (int)round(at_time) << " ms ---\n";
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
        for(auto &p: procs){
            if(p.cpu_consumed>100 && p.remaining>50){
                cout<<"Hotspot detected: P"<<p.pid<<" (cpu_ms="<<(int)round(p.cpu_consumed)<<", rem="<<(int)round(p.remaining)<<"ms)\n";
                cout<<"Suggestion: consider lowering priority or parallelizing workload.\n";
            }
        }
        auto lr = Analyzer::linear_regression(mem_usage_ts, 10);
        double slope = lr.first;
        cout << "Memory slope = "<< slope <<" kb/ms. Forecast in 500ms = "<< (int)round((mem_usage_ts.empty()?0:mem_usage_ts.back().value) + slope*500) <<" kb\n";
        if((mem_usage_ts.empty()?0:mem_usage_ts.back().value) + slope*500 > 1024*1024) cout<<"Warning: projected memory > 1GB, suggest reduce working set or enable swap.\n";
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
        ifstream ifs(argv[1]);
        if(!ifs){ cerr<<"Cannot open "<<argv[1]<<"\n"; return 1; }
        double a,b,m,io;
        while(ifs>>a>>b>>m>>io) jobs.emplace_back(a,b,m,io);
    } else { jobs = sample_jobs(); cout<<"No trace file given — using sample jobset.\n"; }
    Simulator sim; sim.load(jobs); sim.run_and_analyze();
    cout<<"\nSimulation finished.\n";
    return 0;
}
