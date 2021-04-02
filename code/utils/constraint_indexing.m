function iG = constraint_indexing(iG_,iif)

  iG1 = @(fieldname,interval) vertcat(iG_.(fieldname){interval});
  iG = @(varargin) iif( nargin == 1, @() iG1(varargin{1},':'), ...
                        nargin == 2, @() iG1(varargin{1:2}) ...
                      );

end