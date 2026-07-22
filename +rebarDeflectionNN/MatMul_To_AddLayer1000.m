classdef MatMul_To_AddLayer1000 < nnet.layer.Layer & nnet.layer.Formattable
    % A custom layer auto-generated while importing an ONNX network.

    %#ok<*PROPLC>
    %#ok<*NBRAK>
    %#ok<*INUSL>
    %#ok<*VARARG>
    properties (Learnable)
        StatefulPartition_2
        StatefulPartition_7
        StatefulPartition_11
    end

    properties (State)
    end

    properties
        Vars
        NumDims
    end




    methods
        function this = MatMul_To_AddLayer1000(name)
            this.Name = name;
            this.NumInputs = 2;
            this.OutputNames = {'output_0'};
        end

        function [output_0] = predict(this, inputs, inputsNumDims)
            if isdlarray(inputs)
                inputs = stripdims(inputs);
            end
            inputsNumDims = numel(inputsNumDims);
            inputs = rebarDeflectionNN.ops.permuteInputVar(inputs, ['as-is'], 2);

            [output_0, output_0NumDims] = MatMul_To_AddGraph1000(this, inputs, inputsNumDims, false);
            output_0 = rebarDeflectionNN.ops.permuteOutputVar(output_0, ['as-is'], 2);

            output_0 = dlarray(single(output_0), repmat('U', 1, max(2, output_0NumDims)));
        end

        function [output_0] = forward(this, inputs, inputsNumDims)
            if isdlarray(inputs)
                inputs = stripdims(inputs);
            end
            inputsNumDims = numel(inputsNumDims);
            inputs = rebarDeflectionNN.ops.permuteInputVar(inputs, ['as-is'], 2);

            [output_0, output_0NumDims] = MatMul_To_AddGraph1000(this, inputs, inputsNumDims, true);
            output_0 = rebarDeflectionNN.ops.permuteOutputVar(output_0, ['as-is'], 2);

            output_0 = dlarray(single(output_0), repmat('U', 1, max(2, output_0NumDims)));
        end

        function [output_0, output_0NumDims1001] = MatMul_To_AddGraph1000(this, inputs, inputsNumDims, Training)

            % Execute the operators:
            % MatMul:
            [StatefulPartition_3, StatefulPartition_3NumDims] = rebarDeflectionNN.ops.onnxMatMul(inputs, this.StatefulPartition_2, inputsNumDims, this.NumDims.StatefulPartition_2);

            % Add:
            StatefulPartition_1 = StatefulPartition_3 + this.Vars.StatefulPartitionedC;
            StatefulPartition_1NumDims = max(StatefulPartition_3NumDims, this.NumDims.StatefulPartitionedC);

            % Relu:
            StatefulPartition_4 = relu(dlarray(StatefulPartition_1));
            StatefulPartition_4NumDims = StatefulPartition_1NumDims;

            % MatMul:
            [StatefulPartition_8, StatefulPartition_8NumDims] = rebarDeflectionNN.ops.onnxMatMul(StatefulPartition_4, this.StatefulPartition_7, StatefulPartition_4NumDims, this.NumDims.StatefulPartition_7);

            % Add:
            StatefulPartition_6 = StatefulPartition_8 + this.Vars.StatefulPartition_5;
            StatefulPartition_6NumDims = max(StatefulPartition_8NumDims, this.NumDims.StatefulPartition_5);

            % Relu:
            StatefulPartition_9 = relu(dlarray(StatefulPartition_6));
            StatefulPartition_9NumDims = StatefulPartition_6NumDims;

            % MatMul:
            [StatefulPartition_12, StatefulPartition_12NumDims] = rebarDeflectionNN.ops.onnxMatMul(StatefulPartition_9, this.StatefulPartition_11, StatefulPartition_9NumDims, this.NumDims.StatefulPartition_11);

            % Add:
            output_0 = StatefulPartition_12 + this.Vars.StatefulPartition_10;
            output_0NumDims = max(StatefulPartition_12NumDims, this.NumDims.StatefulPartition_10);

            % Set graph output arguments
            output_0NumDims1001 = output_0NumDims;

        end

    end

end
