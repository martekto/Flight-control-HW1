function plotStepResponse(T,Y,systemType,modeName)

global plot_colors

switch systemType
    case {"full", "Full"}
        switch modeName
            case {"short period","SP"}
                figure(1);
                label_y = ["$q [^\circ/s]$","$\alpha [^\circ]$"];
                for i = 1:2
                    subplot(2,1,i); grid on; hold all
                    ylabel(label_y(i),'Interpreter','latex','FontSize',12);
                    plot(T,Y(:,i),'LineWidth',1.5,'Color',plot_colors(1,:));
                end
        
            case {"dutch roll","DR"}
                figure(2);
                label_y = ["$r [^\circ/s]$","$\beta [^\circ]$"];
                for i = 1:2
                    subplot(2,1,i); grid on; hold all
                    ylabel(label_y(i),'Interpreter','latex','FontSize',12);
                    plot(T,Y(:,i),'LineWidth',1.5,'Color',plot_colors(1,:));
                end

            otherwise
                error("Unexpected mode name.")
        end

    case{"approximation"}
        switch modeName
            case {"short period","SP"}
                figure(1);
                for i = 1:2
                    subplot(2,1,i);
                    plot(T,Y(:,i),'LineWidth',1.5, ...
                         'Color',plot_colors(1,:), ...
                         'LineStyle','--');
                end
                xlabel('$t [s]$','Interpreter','latex','FontSize',12);
                hold off
                grid on
                subplot(2,1,1); title('Response to an elevator deflection $\delta_e$ of $20^\circ$', ...
                                      'Interpreter','latex', ...
                                      'FontSize',14);
                lgd = legend('longitudinal','short-period');
                lgd.Interpreter = 'latex'; lgd.FontSize = 12;
                lgd.Location = 'southeast'; lgd.NumColumns = 2;

            case {"dutch roll","DR"}
                figure(2);
                for i = 1:2
                    subplot(2,1,i);
                    plot(T,Y(:,i),'LineWidth',1.5, ...
                         'Color',plot_colors(1,:), ...
                         'LineStyle','--');
                end
                xlabel('$t [s]$','Interpreter','latex','FontSize',12);
                hold off
                grid on
                subplot(2,1,1); title('Response to a ruder deflection $\delta_r$ of $20^\circ$', ...
                                      'Interpreter','latex', ...
                                      'FontSize',14);
                subplot(2,1,2); lgd = legend('lateral','dutch roll');
                lgd.Interpreter  = 'latex'; lgd.FontSize = 12;
                lgd.Location = 'southeast'; lgd.NumColumns = 2;

            otherwise
                error("Unexpected mode name.")
end

end