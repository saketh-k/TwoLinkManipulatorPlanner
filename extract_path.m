function P=extract_path(parent,v_goal)
P=[v_goal]; %(line 1 on pseudo code)
u=v_goal; %(line 2 on pseudo code)
while parent(u)~=u  %(line 3 on pseudo code)
    u=parent(u);  %(line 4 on pseudo code)
    P=[u P]; %(line 5 on pseudo code)
    if u==-1 %for discounected graphs
        P=[];
        break
    end
end
end