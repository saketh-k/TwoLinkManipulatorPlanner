clc;clear;

% clear all




% Generate the collision-free configuration space
obs = [0.5 0.25 0.25 0.25; 
       0 -1.40 0.25 0.35];
len1 = 1;
len2 = 0.5;
drawArmAndObstacles(len1, len2, 0, pi/6, obs);
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
r=2*pi/resolution+0.05; %neighborhood radius (specify this value)
Adj_table={};
node_set=(1:1:n_samples);
computeDistanceOnCircle = @(a,b) min(abs(a-b),2*pi-abs(a-b));
%computeDistanceOnCircle = @(a,b) min(mod(a-b,2*pi),mod(b-a,2*pi));
computeDistanceOnTorus = @(a1,a2,b1,b2) sqrt(computeDistanceOnCircle(a1,a2)^2+computeDistanceOnCircle(b1,b2)^2);

for i=1:n_samples
    Adj_table{i}=[];%Adj_table of node i is initialized empty
    for j=setdiff(node_set,[i])
        %distance = (Samples(i,1)-Samples(j,1))^2+(Samples(i,2)-Samples(j,2))^2;
        distance = computeDistanceOnTorus(Samples(i,1),Samples(j,1),Samples(i,2),Samples(j,2));
        if (distance<=r^2 || (Samples(i,1)-Samples(j,1))^2+(Samples(i,2)-Samples(j,2))^2 <= r^2)
            line([Samples(i,1) Samples(j,1)],[Samples(i,2) Samples(j,2)],'Color','r')
            Adj_table{i}=union(Adj_table{i},[j]);
            edge_length(i,j) = computeDistanceOnTorus(Samples(i,1),Samples(j,1),Samples(i,2),Samples(j,2));
            % edge_length(i,j)=sqrt((Samples(i,1)-Samples(j,1))^2+(Samples(i,2)-Samples(j,2))^2);            
        end
    end
    clear temp n_samples_minus_i j
end

fig_start_goal_connected=figure();
fig_PRMChil = fig_PRM.Children; 
copyobj(fig_PRMChil, fig_start_goal_connected);




start_point = [pi/2, pi/2];
goal_point = [0, 1];

plot(start_point(1,1),start_point(1,2),'m>')
plot(goal_point(1,1),goal_point(1,2),'gs')
Samples(n_samples+1,:)=[start_point];
Samples(n_samples+2,:)=[goal_point];

Adj_table{n_samples+1}=[];%Adj_table of start node is initialized empty
Adj_table{n_samples+2}=[];%Adj_table of goal node  is initialized empty
%connecting to the nodes in r radius of the points
Colour(n_samples+1)='m';
Colour(n_samples+2)='g';
for i=[n_samples+1, n_samples+2]   
    for j=1:n_samples
        if ((Samples(i,1)-Samples(j,1))^2+(Samples(i,2)-Samples(j,2))^2<=r^2)
            line([Samples(i,1) Samples(j,1)],[Samples(i,2) Samples(j,2)],'Color',Colour(i),'LineWidth',3)
            edge_length(i,j)=sqrt((Samples(i,1)-Samples(j,1))^2+(Samples(i,2)-Samples(j,2))^2);
            edge_length(j,i)=edge_length(i,j);
            Adj_table{i}=union(Adj_table{i},[j]);
            Adj_table{j}=union(Adj_table{j},[i]); %i is also a neighbor of j
        end
    end
    clear temp n_samples_minus_i
end

fig_shortestpath=figure();
fig_start_goal_connectedChil = fig_start_goal_connected.Children; 
copyobj(fig_start_goal_connectedChil, fig_shortestpath);

[parent,dist]=Dijkstra_search(n_samples+2,Adj_table,edge_length,n_samples+1,n_samples+2);

path=extract_path(parent,n_samples+2)
path_length=dist(n_samples+2)
if length(path)~=0
for k=1:length(path)-1
   line([Samples(path(k),1),Samples(path(k+1),1)],[Samples(path(k),2),Samples(path(k+1),2)],'Color','Y','LineWidth',4)
    hold on
end
end

thetas = [];
for i = 1:size(path,2)
    thetas = [thetas;Samples(path(i),:)];
end

animateArm(len1, len2, thetas, obs);




