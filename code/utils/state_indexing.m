function [iV, iif] = state_indexing(xLabels,uLabels,N,xuI)

nx = numel(xLabels);
nu = numel(uLabels);

offset = 0;
for k = 1:N
    for j = 1:nx
        iV_(k).x.(char(xLabels{j})) = j + offset;
    end
    offset = offset + nx;
    for j = 1:nu
        iV_(k).u.(char(uLabels{j})) = j + offset;
    end
    offset = offset + nu;
end
for j = 1:nx
    iV_(N+1).x.(char(xLabels{j})) = j + offset;
end
for j = 1:nu
    iV_(N+1).u.(char(uLabels{j})) = zeros(0,1);
end

iV1 = @(fieldname,interval) reshape(cell2mat(arrayfun(@(x) ...
        struct2array(x.(fieldname)),iV_(interval),...
        'UniformOutput',false)).',[],1);

iV2 = @(fieldname,interval,secondname) reshape(cellfun(@(y) ...
          y(xuI.(fieldname).(secondname)), arrayfun(@(x) ...
          struct2array(x.(fieldname)),iV_(interval),...
          'UniformOutput',false)),[],1);

iif = @(varargin) varargin{2 * find([varargin{1:2:end}], 1, 'first')}();

iV = @(varargin) iif( nargin == 1, @() iV1(varargin{1},':'), ...
                      nargin == 2, @() iV1(varargin{1:2}), ...
                      nargin == 3, @() iV2(varargin{1:3})...
                      );
                    
end