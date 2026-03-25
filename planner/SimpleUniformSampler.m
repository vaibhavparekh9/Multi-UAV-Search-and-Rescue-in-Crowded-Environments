classdef SimpleUniformSampler < handle
    % Minimal uniform sampler that works with ExampleHelperUAVStateSpace,
    % trying several likely names for the bounds property/method.

    properties
        StateSpace
        Bounds   % [numVars x 2]
    end

    methods
        function obj = SimpleUniformSampler(ss)
            obj.StateSpace = ss;
            obj.Bounds = SimpleUniformSampler.tryGetBounds(ss); % <-- robust lookup
        end

        function X = sample(obj, n)
            if nargin < 2, n = 1; end

            % If the state space already knows how to sample uniformly, use it.
            if ismethod(obj.StateSpace, 'sampleUniform')
                X = obj.StateSpace.sampleUniform(n);
                return;
            end

            % Otherwise sample from the discovered bounds.
            B  = obj.Bounds;           % [numVars x 2]
            lo = B(:,1)'; hi = B(:,2)';  % 1 x D
            X  = rand(n, numel(lo)) .* (hi - lo) + lo;

            % Clamp to bounds if supported
            if ismethod(obj.StateSpace, 'enforceStateBounds')
                X = obj.StateSpace.enforceStateBounds(X);
            end
        end
    end

    methods (Static, Access=private)
        function B = tryGetBounds(ss)
            % Try common property names first
            candProps = {'Bounds', 'StateBounds', 'RandomStateBounds', 'DefaultRandomStateBounds'};
            for k = 1:numel(candProps)
                p = candProps{k};
                if isprop(ss, p)
                    B = ss.(p);
                    return;
                end
            end
            % Try common methods
            candMeth = {'getBounds','bounds','stateBounds','randomStateBounds','defaultRandomStateBounds'};
            for k = 1:numel(candMeth)
                m = candMeth{k};
                if ismethod(ss, m)
                    B = ss.(m);
                    return;
                end
            end
            error(['Could not determine state-space bounds. Your ExampleHelperUAVStateSpace ' ...
                   'should expose a bounds property/method (e.g., ''Bounds'' or ''RandomStateBounds'').']);
        end
    end
end