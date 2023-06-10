first_location = start_point;
for i=0:(length(label_location)-1)
    thetas1 = findPath(Samples, n_samples, Adj_table, r, edge_length, xyToTheta(first_location, len1, len2), xyToTheta(label_location(i+1,:), len1, len2));
    all_thetas = [all_thetas' thetas1']';
    first_location = label_location(i+1,:);
end
thetas4 = findPath(Samples, n_samples, Adj_table, r, edge_length, xyToTheta(label_location(end,:), len1, len2),xyToTheta(goal_point, len1,len2));
toc
animateArm(len1, len2, [all_thetas], obs, label_location);