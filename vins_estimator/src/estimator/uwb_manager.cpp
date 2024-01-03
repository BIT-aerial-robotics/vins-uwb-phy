#include "uwb_manager.h"



OdometryVins OdometryVins::interpolation(OdometryVins nx,double t)
{
    OdometryVins tmp;
    tmp.time=t;
    double t1=time,t2=nx.time;
    tmp.Vs=Vs+((Vs-nx.Vs)/(t2-t1))*(t-t1);
    //ROS_INFO("%lf %lf %lf %lf %lf %lf %lf",t1,t2,t,Ps.x(),nx.Ps.x(),Vs.x(),nx.Vs.x());
    tmp.Ps=Ps+0.75*((Ps-nx.Ps)/(t2-t1))*(t-t1)+0.25*(tmp.Vs+Vs)*0.5*(t-t1);

    tmp.Ws=Ws+((Ws-nx.Ws)/(t2-t1))*(t-t1);
    tmp.Rs = Rs;
    tmp.Rs=tmp.Rs.slerp((t - t1) / (t2 - t1), nx.Rs);
    return tmp;
}
OdometryVins OdometryVins::predict(double t)
{
    OdometryVins tmp;
    tmp.time=t;
    tmp.Vs=Vs;
    tmp.Ps=Ps+Vs*(t-time);
    tmp.Ws=Ws;
    tmp.Rs =Rs;
    Eigen::Quaterniond delta;
    delta.x()=tmp.Ws.x()*(t-time)*0.5;
    delta.y()=tmp.Ws.y()*(t-time)*0.5;
    delta.z()=tmp.Ws.z()*(t-time)*0.5;
    delta.w()=0;
    //delta.w()=sqrt(1-delta.x()*delta.x()+delta.y()*delta.y()+delta.z()*delta.z());
    //tmp.Rs=tmp.Rs*delta;
    tmp.Rs.normalize();
    return tmp;
}
UWBManager::UWBManager(){
    uwb_num=0;
    uwb_loc.resize(0);
    clearState();
}
UWBManager::UWBManager(int _uwb_num,vector<Eigen::Vector3d> _uwb_loc)
{
    uwb_num=_uwb_num;
    uwb_loc=_uwb_loc;
    uwb_range_window=vector<deque<UWBMeasurement>>(uwb_num+1,deque<UWBMeasurement>());
    //uwb_range_window.resize(uwb_num);
    //uwb_range_sol_data.resize(uwb_num);
    uwb_range_sol_data=vector<vector<UWBMeasurement>>(uwb_num+1,vector<UWBMeasurement>(0));
    clearState();
    // for(int i=0;i<uwb_num;i++){
    //     vector<UWBMeasurement>tmp;
    //     uwb_range_raw_data.push_back(
    //         vector<>
    //     )
    // }
}
void UWBManager::addOdometryMeasurements(Eigen::Vector3d Ps,Eigen::Vector3d Vs,Eigen::Vector3d Ws,Eigen::Quaterniond Rs,double time)
{
    odometry.push_back(OdometryVins(Ps,Vs,Ws,Rs,time));
}
void UWBManager::clearState()
{
    sigma=0.1;
    sample=15;
    RANGE_SIZE=7;
    OFFSET_THRESH=0.04;
    publish_smooth_range=true;
    bf=new ButterworthLowPassFilter(5.0,50.0,2);
    for(int i=0;i<=9;i++)pre_sum[i]=0.0,range_len[i]=0,last_smooth_idx[i]=0;
}
double UWBManager::accumulate(int idx)
{
    double sum=0;
    for (int i = 0; i < uwb_range_window[idx].size(); i++) {
        sum+=uwb_range_window[idx][i].range;
    }
    return sum/(int)(uwb_range_window[idx].size());
}

bool UWBManager::query(OdometryVins &x,double time){
    if(odometry.size()==0)return false;
    auto iter=lower_bound(odometry.begin(),odometry.end(),time,comp());
    if(abs(iter->time-time)>0.75)return false;
    if(iter==odometry.end())return false;
    if(iter!=odometry.begin()){
        auto last=prev(iter);
        OdometryVins tmp=*last;
        x=tmp.interpolation(*iter,time);
        return true;
    }
    else{
        if(abs(iter->time-time)>0.15)return false;
        OdometryVins tmp=*iter;
        x=tmp.predict(time);
        return true;
    }
}
bool UWBManager::addUWBMeasurements(int idx,double time,double range){
    UWBMeasurement tmp(idx,range,time);
    if(uwb_range_window[idx].size()==0){

        //printf("size==%d\n",uwb_range_window[idx].size());
        //std::cout<<uwb_range_window[idx]<<std::endl;
        uwb_range_window[idx].push_back(tmp);
        uwb_range_sol_data[idx].push_back(tmp);
        //printf("len not enough\n");
        smoothRange(idx);
        return true;
    }
    else{
        if(range!=uwb_range_sol_data[idx].back().range){
            OdometryVins x1,x2;
            bool q1,q2;
            //q1=query(x1,uwb_range_window[idx].back().time);
            //q2=query(x2,time);
            //ROS_INFO("new uwb data");
            double offset = range - accumulate(idx);
            double offset_to_last = range - uwb_range_sol_data[idx].back().range;
            //printf("offset %lf  offset_to_last %lf len=%d\n",offset,offset_to_last,uwb_range_window[idx].size());
            if(q1){
                double delta_t=time-uwb_range_window[idx].back().time;
                double delta_p;
                if(q2)delta_p=(x1.Ps-x2.Ps).norm();
                else delta_p=x1.Vs.norm()*(time-uwb_range_window[idx].back().time);
                if(abs(offset_to_last)>exp(1.1+delta_t)*delta_p){
                    range_updated=false;
                    now_offset=0;
                    return false;
                }
            }

            // if( abs(offset) > OFFSET_THRESH && abs(offset_to_last) > OFFSET_THRESH)
            // {
            //     now_offset=offset;
            //     offset_time=ros::Time::now().toSec();
            //     range_updated = true;
            // }
            // else if(abs(offset)>OFFSET_THRESH)
            // {
            //     uwb_range_window[idx].push_back(UWBMeasurement(idx,time,range-now_offset));
            //     uwb_range_window[idx].pop_front();
            //     range_updated = true;
            // }
            // else if(offset_to_last<-OFFSET_THRESH){
            //     now_offset=0;
            // }
            // else{
            //     uwb_range_window[idx].push_back(UWBMeasurement(idx,time,range-now_offset));
            //     uwb_range_window[idx].pop_front();
            //     range_updated = true;
            //     now_offset=0;
            // }
            // if(ros::Time::now().toSec()-offset_time>1000){
            //     now_offset=0;
            //     range_updated=false;
            // }
            // if(time-uwb_range_window[idx].back().time>1000){
            //     uwb_range_window.clear();
            //     now_offset=0;
            //     range_updated=false;

            // }
            if(abs(offset_to_last)<2*OFFSET_THRESH){
                if((uwb_range_window[idx].size()>=4&&abs(offset)>4*OFFSET_THRESH))
                range_updated=false;
                else
                range_updated=true;
            }
            //range_updated=true;
            if(abs(OFFSET_THRESH-0.6)<=0.01){
                range_updated=true;
                while(uwb_range_window[idx].size()>0)uwb_range_window[idx].pop_front();
                OFFSET_THRESH=0.04;
            }
            bool res=range_updated;
            if(range_updated){
                uwb_range_sol_data[idx].push_back(tmp);
                uwb_range_window[idx].push_back(tmp);
                if(uwb_range_window[idx].size()>RANGE_SIZE)uwb_range_window[idx].pop_front();
                OFFSET_THRESH=0.04;
                smoothRange(idx);
            }
            else{
                OFFSET_THRESH=min(0.6,OFFSET_THRESH+0.014);
            }
            range_updated=false;
            return res;
        }
        else{
            printf("%d %lf %lf",idx,time,range);
            return false;
        }
        
    }
    

}

void UWBManager::smoothRange(int idx)
{
    //printf("smooth range %d %d %d",idx,uwb_range_sol_data[idx].size(),last_smooth_idx[idx]);
    
    // int len=150+sample/2+sample/2;
    // int odd=sample/2;
    // int measurement_id=odd;
    // vector<UWBMeasurement> tmp(len);

    if(1)
    {
        int ava=0;
        int len=4;
        if(ava==0)len=uwb_range_window[idx].size();
        else len=6;
        //if(uwb_range_sol_data[idx].size()<=len)return;
        vector<double> out(len,0);
        double s=0;
        int fd=0;
        for(int i=len-1;i>=0;i--){
            if(ava==1){
                out[i]=1.00;
            }
            else{
                out[i]=((1.00)/(sqrt(2*3.14*sigma)))*exp(-(i-len+1.00)*(i-len+1.00)/(sigma*sigma));
            }
            if(uwb_range_window[idx].back().time-uwb_range_window[idx][i].time>(len-1-i)*0.06){
                out[i]=0;
            }
            // if(uwb_range_sol_data[idx][uwb_range_sol_data[idx].size()-1].time-uwb_range_sol_data[idx][uwb_range_sol_data[idx].size()-1-fd].time>(fd+1)*4*0.02){
            //     out[i]=0;
            //     break;
            // }
            fd++;
            s+=out[i];
        }
        for(int i=0;i<len;i++)out[i]/=s;
        double sp=0;
        fd=0;
        // for(int i=uwb_range_sol_data[idx].size()-len;i<uwb_range_sol_data[idx].size();i++)
        // {
        //     sp+=out[fd++]*uwb_range_sol_data[idx][i].range;
        // }
        for(int i=0;i<len;i++)
        {
            sp+=out[i]*uwb_range_window[idx][i].range;
        }
        uwb_range_sol_data[idx][uwb_range_sol_data[idx].size()-1].range=sp;
        uwb_range_window[idx][uwb_range_window[idx].size()-1].range=sp;
    }
    else{
        double filteredOutput = bf->filter(uwb_range_sol_data[idx].back().range);
        printf(" filteredOutput = %lf\n",filteredOutput);
        uwb_range_sol_data[idx][uwb_range_sol_data[idx].size()-1].range=filteredOutput;
    }
    // for(int i=uwb_range_sol_data[idx].size()){
    //     tmp[measurement_id++]=uwb_range_sol_data[idx][i];
    // }
    // for(int i=0;i<odd;i++)
    // tmp[i]=tmp[odd];
    // for(int i=len-1;i>=len-odd;i++)
    // tmp[i]=tmp[len-odd-1];
    // gaussSmoothen(tmp,out);
    // measurement_id=0;
    // for(int i=odd;i<len-odd;i++){
    //     uwb_range_sol_data[idx][measurement_id++].range=out[i];
    // }
    //last_smooth_idx[idx]=uwb_range_sol_data[idx].size();
}
void UWBManager::gaussKernel(vector<double>&kernal)
{
    int kernel_id=0;
    int steps = (sample - 1) / 2;
    double stepSize = (3 * sigma) / steps;
    for (int i = steps; i >= 1; i--) {
        kernal[kernel_id++]=Utility::gauss(sigma, i * stepSize * -1);
    }
    kernal[kernel_id++]=Utility::gauss(sigma, 0);
    for (int i = 1; i <= steps; i++) {
        kernal[kernel_id++]=Utility::gauss(sigma, i * stepSize);
    }
}
void UWBManager::gaussSmoothen(vector<UWBMeasurement>&values,vector<double>&out)
{
    vector<double>kernel(15,0.0);
    gaussKernel(kernel);
    int sampleSide = sample / 2;
    int valueIdx = sample / 2 + 1;
    unsigned long ubound = values.size();
    for (unsigned long i = 0; i < ubound; i++) {
        double sample = 0;
        int sampleCtr = 0;
        //std::cout << "Now at value" << i << ": ";
        for (long j = i - sampleSide; j <= i + sampleSide; j++) {
            //std::cout << j << " ";
            if (j > 0 && j < ubound) {
                int sampleWeightIndex = sampleSide + (j - i);
                //std::cout << "(" << sampleWeightIndex << " [" << kernel[sampleWeightIndex] << "]) ";
                sample += kernel[sampleWeightIndex] * values[j].range;
                sampleCtr++;
            }
        }
        double smoothed = sample / (double)sampleCtr;
        //std::cout << " S: " << sample << " C: " << sampleCtr << " V: " << values[i] << " SM: " << smoothed << std::endl;
        out[i]=smoothed;
    }
}