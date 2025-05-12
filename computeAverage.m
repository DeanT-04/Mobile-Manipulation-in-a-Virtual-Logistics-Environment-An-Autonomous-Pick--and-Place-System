function [averageX, averageY] = computeAverage(frameXCoordinates, frameYCoordinates)
    % computeAverage Computes the average of X and Y coordinates from cell arrays.
    %
    % Args:
    %   frameXCoordinates (cell): A cell array where each cell contains a numeric array of X coordinates.
    %   frameYCoordinates (cell): A cell array where each cell contains a numeric array of Y coordinates.
    %
    % Returns:
    %   averageX (double): The mean of all X coordinates. NaN if input is empty.
    %   averageY (double): The mean of all Y coordinates. NaN if input is empty.

    % Concatenate all coordinates from the cell arrays
    allXCoordinates = [frameXCoordinates{:}];
    allYCoordinates = [frameYCoordinates{:}];

    % Check if the concatenated arrays are empty
    if isempty(allXCoordinates)
        averageX = NaN;
        averageY = NaN;
    else
        averageX = mean(allXCoordinates);
        averageY = mean(allYCoordinates);
    end
end