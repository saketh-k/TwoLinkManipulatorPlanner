function thetas = findPath(Samples, n_samples, Adj_table, r, edge_length, fig_start_goal_connected, start_point, goal_point)

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
end