%%
clc,clear
bag = rosbag('s_curve2.bag');
resolution=2*pi/720;
geometry_message=select(bag,'MessageType','sensor_msgs/LaserScan');
data=readMessages(geometry_message); 

LaserData=double(data{1,1}.Ranges(:));



figure;
title("Original LIDAR Data")

for i=1:1:720
    if i < 360
     polarscatter(-(360-i)*resolution,LaserData(i),'.')
    LaserData(i,2)=-(360-i)*resolution;
     hold on
    elseif i>= 360
     polarscatter((i-360)*resolution,LaserData(i),'.')    
    LaserData(i,2)=(i-360)*resolution;
    hold on
    end
end

minNum = 5;
r = 0.4 ;
N =1;
flag =0;
tempN=1;
ncluster=[];
for i=1:1:719
    if (LaserData(i+1,1)*cos(LaserData(i+1,2))-LaserData(i,1)*cos(LaserData(i,2)))^2+(LaserData(i+1,1)*sin(LaserData(i+1,2))-LaserData(i,1)*sin(LaserData(i,2)))^2 <= (r)^2

        if flag==0
            N=N+1;
            ncluster(N,tempN)=i;
            flag=1;
        end
        ncluster(N,tempN)=i;
        tempN=tempN+1;
    
    else
        flag=0;
        tempN=1;
    end
end
[m,n]=size(ncluster);

color=['b*','y.','gv','r^','black*','b.','rv','y^',];
figure;
title("After Clustering")
for i=1:1:m
    for j=1:1:n
        if ncluster(i,j)~=0
           polarscatter(LaserData(ncluster(i,j),2),LaserData(ncluster(i,j),1),color(mod(i,8)+1))
           hold on
        end
    end
end
