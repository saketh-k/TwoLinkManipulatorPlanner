%clc;clear;
clear all

label_location=[1.25 sqrt(3)/4;
                0.8    0.75
                0.6    0.6
                -sqrt(2)/4  -1-sqrt(2)/4
                -0.5 -0.5]

% Generate the collision-free configuration space
obs = [0.5 0.25 0.25 0.25; 
       0 -1.40 0.25 0.35];
len1 = 1;
len2 = 0.5;
drawArmAndObstacles(len1, len2, 0, pi/6, obs, label_location);
[C, th1, th2] = configSpacePlot(1, 0.5, obs);

[grid1, grid2] = ndgrid(th1, th2);
resolution = 50;
n_samples = resolution^2;
grid1 = grid1.*not(C);
grid2 = grid2.*not(C);
Samples = [grid1(:), grid2(:)];

for i = size(Samples,1):-1:1
    if Samples(i,:) == [0,0]
        Samples(i,:) = [];
    end
end
Samples(size(Samples,1),:) = [0,0];
n_samples = size(Samples,1);


fig_samples=figure;
plot(Samples(:,1),Samples(:,2),'.') 
xlim([0,2*pi])
ylim([0,2*pi])
axis square

fig_PRM=figure();
fig_samplesChil = fig_samples.Children; 
copyobj(fig_samplesChil, fig_PRM);

% [adjacency, weights] = generateGraph(C, th1, th2);

hold on
edge_length=-1*ones(n_samples+2,n_samples+2);%we initialize this matrix -1; at the end of process if an element is -1 it means that the corresponding elements are not connected.
r=2.78*2*pi/resolution+0.01; %neighborhood radius (specify this value)
Adj_table={};
node_set=(1:1:n_samples);
computeDistanceOnCircle = @(a,b) min(abs(a-b),2*pi-abs(a-b));
%computeDistanceOnCircle = @(a,b) min(mod(a-b,2*pi),mod(b-a,2*pi));
computeDistanceOnTorus = @(a1,a2,b1,b2) sqrt(computeDistanceOnCircle(a1,a2)^2+computeDistanceOnCircle(b1,b2)^2);

parfor i=1:n_samples
    Adj_table{i}=[];%Adj_table of node i is initialized empty
    for j=1:n_samples
        if j==i
            continue
        end
        %distance = (Samples(i,1)-Samples(j,1))^2+(Samples(i,2)-Samples(j,2))^2;
        distance = computeDistanceOnTorus(Samples(i,1),Samples(j,1),Samples(i,2),Samples(j,2));
        if (distance<=r^2 )
            % line([Samples(i,1) Samples(j,1)],[Samples(i,2) Samples(j,2)],'Color','r')
            Adj_table{i}=union(Adj_table{i},[j]);
            edge_length(i,j) = computeDistanceOnTorus(Samples(i,1),Samples(j,1),Samples(i,2),Samples(j,2));
            % edge_length(i,j)=sqrt((Samples(i,1)-Samples(j,1))^2+(Samples(i,2)-Samples(j,2))^2);            
        end
    end
end
clear temp n_samples_minus_i j

% fig_start_goal_connected=figure();
% fig_PRMChil = fig_PRM.Children; 
% copyobj(fig_PRMChil, fig_start_goal_connected);
% 
% fig_start_goal_connected2=figure();
% fig_PRMChil2 = fig_PRM.Children; 
% copyobj(fig_PRMChil2, fig_start_goal_connected);

start_point = [0.1, 0.125];
goal_point = [5.2, 1];
goal2_point = [2, 5];

% thetas1 = findPath(Samples, n_samples, Adj_table, r, edge_length, start_point, goal_point);
% all_thetas = thetas1;
% 
% thetas2 = findPath(Samples, n_samples, Adj_table, r, edge_length, goal_point, start_point);
tic
all_thetas = [];
first_location = start_point;
for i=0:(length(label_location)-1)
    thetas1 = findPath(Samples, n_samples, Adj_table, r, edge_length, first_location, label_location(i+1,:));
    all_thetas = [all_thetas' thetas1']';
    first_location = label_location(i+1,:);
end
thetas4 = findPath(Samples, n_samples, Adj_table, r, edge_length, label_location(end,:),goal_point);
toc
animateArm(len1, len2, [all_thetas], obs, label_location);




