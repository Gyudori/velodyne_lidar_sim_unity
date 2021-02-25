function VisualizeVoxel(VoxelPointCount, voxelSize, xyzCount, xyzMin)

% Visualization parameters - cube 
v = voxelSize;
[X,Y] = meshgrid([0 0 v v 0]-v/2,[0 0 v v]-v/2);    
Z     = zeros(size(X))-v/2;
Z([6 7 10 11]) = v/2;

% Colormap
cmap = parula(100);  

% Voxel colormap value
VoxelCmap = round((VoxelPointCount / max(max(max(VoxelPointCount)))) * 100);
VoxelCmap(VoxelCmap == 0) = 1;

% Plotting
for i = 1:xyzCount(1)
    for j = 1:xyzCount(2)
        for k = 1:xyzCount(3)
            if(VoxelPointCount(i, j, k) ~= 0) 
                x1 = xyzMin(1) + (i-1/2)*voxelSize;
                y1 = xyzMin(2) + (j-1/2)*voxelSize;
                z1 = xyzMin(3) + (k-1/2)*voxelSize;
                hold on
                surf(X+x1,Y+y1,Z+z1,'facecolor',cmap(VoxelCmap(i, j, k),:), 'LineStyle', 'none')
            end
        end
    end
end
hold off; axis equal; grid on; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');


