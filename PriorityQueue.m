% --- Priority Queue for A* ---
% Simple implementation using cell array and sorting (less efficient than
% java.util.PriorityQueue but avoids Java dependency if needed)
classdef PriorityQueue < handle
    properties
        elements % Cell array {priority, item}
    end
    methods
        function obj = PriorityQueue()
            obj.elements = {};
        end
        function insert(obj, item, priority)
            obj.elements{end+1} = {priority, item};
            % Keep sorted (simple insertion sort idea)
             idx = length(obj.elements);
             while idx > 1 && obj.elements{idx}{1} < obj.elements{idx-1}{1}
                 temp = obj.elements{idx};
                 obj.elements{idx} = obj.elements{idx-1};
                 obj.elements{idx-1} = temp;
                 idx = idx - 1;
             end
        end
        function item = extractMin(obj)
            if obj.isEmpty()
                error('Priority queue is empty');
            end
            item = obj.elements{1}{2};
            obj.elements(1) = []; % Remove first element
        end
         function decreaseKey(obj, item, newPriority)
             found = false;
             for i = 1:length(obj.elements)
                 if isequal(obj.elements{i}{2}, item)
                     % Check if new priority is actually lower
                     if newPriority < obj.elements{i}{1}
                         obj.elements{i}{1} = newPriority;
                         % Resorting needed - simple resort for now
                         [~, sortIdx] = sort(cellfun(@(x) x{1}, obj.elements));
                         obj.elements = obj.elements(sortIdx);
                     end
                     found = true;
                     break;
                 end
             end
             if ~found
                 % Item not found, could insert but standard decreaseKey assumes it exists
                 % warning('Item not found in PriorityQueue for decreaseKey');
             end
         end
        function tf = isEmpty(obj)
            tf = isempty(obj.elements);
        end
    end
end
