function [parent,dist]=Dijkstra_search(n,Adj_table,weight,v_start,v_goal)
%n is the number of the nodes in the graph
%Adj_table adjacency table of the graph (assumes this table is defined as a matlab cell)
%weight: efge weight matrix
%v_start: start node label
%v_goal: goal node label
for i=1:n   %(line 1 on pseudo code)
    dist(i)=inf;%(line 2 on pseudo code)
    parent(i)=-1; %(line 3 on pseudo code) %-1 is equivalent of none, it means that node i does not have a parent
end
dist(v_start)=0; %(line 4 on pseudo code)
parent(v_start)=v_start; %(line 5 on pseudo code)
Q=(1:1:n); %(line 6 on pseudo code)
while length(Q)~=0 %(line 7 on pseudo code)
    dist_Q=dist(Q);
    [min_dist,index_min]= min(dist_Q);
    v=Q(index_min); %(line 8 on pseudo code) 
    dist_Q=[]; Q(index_min)=[]; %(line 9 on pseudo code)
    for w=Adj_table{v} %(line 10 on pseudo code)
        if dist(w)>dist(v)+weight(v,w) %(line 11 on pseudo code)
            dist(w)=dist(v)+weight(v,w);  %(line 12 on pseudo code)
            parent(w)=v;%(line 12 on pseudo code)
        end 
    end
end
end
