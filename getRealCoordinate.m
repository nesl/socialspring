function [ xy_est ] = getRealCoordinate( file, t )
data = load(file);
times = data.annotated_path(:,1);
xy = data.annotated_path(:,2:3);

% make sure we start from time 0
times = times - times(1);

% find last time in annotated
[i_last] = find(times <= t, 1, 'last');
t_last = times(i_last);

% find next time in annotated
[i_next] = find(times > t, 1, 'first');
t_next = times(i_next);

if isempty(i_next)
    xy_est = xy(i_last,:);
    return;
end

% linear interpolation
xy1 = xy(i_last,:);
xy2 = xy(i_next,:);
vec = xy2 - xy1;

xy_est = xy1 + vec*( (t-t_last)/(t_next-t_last) );

% convert pixels to meters
xy_est = 0.05*xy_est;

end

