function traj3d_plot(N, E, alt, phase)
%TRAJ3D_PLOT Live 3D trajectory visualization (called from Simulink)
    persistent fig hLine hPoint tN tE tAlt count ax
    if isempty(count) || (count > 100 && abs(N) < 1 && abs(E) < 1 && alt < 5)
        count = 0;
        tN = zeros(1, 50000);
        tE = zeros(1, 50000);
        tAlt = zeros(1, 50000);
        if ~isempty(fig) && isvalid(fig), close(fig); end
        fig = []; hLine = []; hPoint = [];
    end
    count = count + 1;
    tN(count) = N;
    tE(count) = E;
    tAlt(count) = alt;

    if mod(count, 100) == 1 || phase >= 15
        if isempty(fig) || ~isvalid(fig)
            fig = figure('Name','3D Trajectory','NumberTitle','off', ...
                         'Position',[100 100 900 700]);
            ax = axes(fig);
            hLine = plot3(ax, tN(1), tE(1), tAlt(1), 'b-', 'LineWidth', 1.5);
            hold(ax, 'on');
            hPoint = plot3(ax, tN(1), tE(1), tAlt(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
            plot3(ax, 0, 0, 0, 'gp', 'MarkerSize', 14, 'MarkerFaceColor', 'g');
            hold(ax, 'off');
            xlabel(ax, 'North (m)'); ylabel(ax, 'East (m)'); zlabel(ax, 'Altitude (m)');
            title(ax, '3D Flight Trajectory');
            grid(ax, 'on'); view(ax, 45, 30);
            daspect(ax, [1 1 1]);
            ax.Color = [0.98 0.98 0.98];
        end
        set(hLine, 'XData', tN(1:count), 'YData', tE(1:count), 'ZData', tAlt(1:count));
        set(hPoint, 'XData', N, 'YData', E, 'ZData', alt);
        daspect(ax, [1 1 1]);
        drawnow limitrate;
    end
end
