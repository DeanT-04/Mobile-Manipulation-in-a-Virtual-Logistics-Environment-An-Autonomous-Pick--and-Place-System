function computeObjectAverages()
    global orangeX orangeY orangeAvgX orangeAvgY
    global bottleX bottleY bottleAvgX bottleAvgY
    global cupX cupY cupAvgX cupAvgY

    [orangeAvgX, orangeAvgY] = computeAverage(orangeX, orangeY);
    [bottleAvgX, bottleAvgY] = computeAverage(bottleX, bottleY);
    [cupAvgX, cupAvgY] = computeAverage(cupX, cupY);
end
