%% Copyright (C) 2019 Antonius R. Burgers
%%
%% This file is part of Octave.
%%
%% Octave is free software: you can redistribute it and/or modify it
%% under the terms of the GNU General Public License as published by
%% the Free Software Foundation, either version 3 of the License, or
%% (at your option) any later version.
%%
%% Octave is distributed in the hope that it will be useful, but
%% WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with Octave; see the file COPYING.  If not, see
%% <https://www.gnu.org/licenses/>.

classdef octaveanimatedline < handle
  properties (Hidden)
    h
  end
  properties
    MaximumNumPoints = 0;
  end

  methods (Static)
    function [name, deflt] = lineprops
      tmp = { ...
        'Color', 'black', ...
        'DisplayName', 'none', ...
        'Marker', 'none', ...
        'MarkerSize', 7, ...
        'MarkerFaceColor', 'black', ...
        'LineWidth', 1};
      tmp = reshape(tmp, 2, []);
      name  = tmp(1, :);
      deflt = tmp(2, :);
    end
  end

  methods (Hidden)
    function truncatepoints(obj)
      [x, y, z] = obj.getpoints;
      if obj.MaximumNumPoints > 0 && numel(x) > obj.MaximumNumPoints
        is = numel(x) - obj.MaximumNumPoints + 1;
        set(obj.h, 'XData', x(is:end));
        set(obj.h, 'YData', y(is:end));
        if ~isempty(z)
          set(obj.h, 'ZData', z(is:end));
        end
      end
    end

    function obj = subsasgn(obj, s, val)
      if strcmp(s.type, '.') && any(strcmpi(s.subs, obj.lineprops))
        set(obj.h, s.subs, val);
      else
        obj = builtin('subsasgn', obj, s, val);
      end
    end

  end

  methods
    function obj = octaveanimatedline(varargin)
      [lineprop_names, lineprop_dflts] = obj.lineprops;

      p = inputParser;
      p.CaseSensitive = false;
      for i = 1 : numel(lineprop_names)
        p.addParameter(lineprop_names{i}, lineprop_dflts{i});
      end
      p.addParameter('MaximumNumPoints', 0);

      have_y = 0;
      have_z = 0;
      is = 1;
      if nargin > 0 && isnumeric(varargin{1})
        x = varargin{1};
        is = 2;
        if nargin > 1 && isnumeric(varargin{2})
          y = varargin{2};
          is = 3;
          have_y = 1;
          if nargin > 2 && isnumeric(varargin{3})
            z = varargin{3};
            is = 4;
            have_z = 1;
          end
        end
      end
      p.parse(varargin{is:end});
      % hold on required, because animatedline used
      % the current axis, and you want to preserve
      % e.g. xlim, ylim settings for those axes
      hold('on');
      if have_z
        obj.h = plot3(gca, x, y, z);
        obj.truncatepoints;
      elseif have_y
        obj.h = plot(gca, x, y);
        obj.truncatepoints;
      else
        obj.h = plot(gca, 0, 0);
        obj.clearpoints;
      end
      hold('off');
      for f = reshape(fieldnames(p.Results), 1, [])
        if any(strcmp(f{1}, lineprop_names))
          set(obj.h, f{1}, p.Results.(f{1}));
        else
          % for MaximumNumPoints
          obj.(f{1}) = p.Results.(f{1});
        end
      end
    end

    function [x, y, z] = getpoints(obj)
      x = get(obj.h, 'XData');
      y = get(obj.h, 'YData');
      z = get(obj.h, 'ZData');
    end

    function clearpoints(obj)
      set(obj.h, 'XData', []);
      set(obj.h, 'YData', []);
      set(obj.h, 'ZData', []);
    end

    function addpoints(obj, x, y, z)
      [xo, yo, zo] = obj.getpoints;
      if exist('z', 'var')
        set(obj.h, 'ZData', [zo, z(:)']);
      end
      set(obj.h, 'XData', [xo, x(:)']);
      set(obj.h, 'YData', [yo, y(:)']);
      obj.truncatepoints;
    end
  end
end
