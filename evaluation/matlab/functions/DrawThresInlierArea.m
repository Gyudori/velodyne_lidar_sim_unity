function DrawThresInlierArea(threshold, LRFlag)


line([threshold, threshold], ylim, 'Linewidth', 2, 'Color', 'r');
xlimVal = xlim;
ylimVal = ylim;
if LRFlag == 'Left'
    alpha(area([xlimVal(1), threshold], [ylimVal(2), ylimVal(2)], 'FaceColor', 'r', 'EdgeColor', 'r', 'LineStyle', 'none'), .3)
elseif LRFlag == 'Right'
    alpha(area([threshold xlimVal(2)], [ylimVal(2), ylimVal(2)], 'FaceColor', 'r', 'EdgeColor', 'r', 'LineStyle', 'none'), .3)
else
    Error('Wrong Flag.');
end