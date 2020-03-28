%rosinit;
actual_pose_sub=rossubscriber("/drone/gt_pose");
contact_sub=rossubscriber("/link_0_contact");X_actual= [];Y_actual=[];Z_actual=[];
X_contact= [];Y_contact=[];Z_contact=[];
X_contact2= [];Y_contact2=[];Z_contact2=[];
FX_contact= [];FY_contact=[];FZ_contact=[];
F_norm = [];
f1 = figure;
f2 = figure;
f3 = figure;
while 1
actual_pose_msg = receive(actual_pose_sub,0.1);
contact_msg = receive(contact_sub,0.1);
X_actual = [X_actual, actual_pose_msg.Position.X];
Y_actual = [Y_actual, actual_pose_msg.Position.Y];
Z_actual = [Z_actual, actual_pose_msg.Position.Z];
figure(f1);
plot3(X_actual,Y_actual,Z_actual,'m','LineWidth',2)
axis equal
xlabel('x[m]')
ylabel('y[m]')
zlabel('z[m]')
title('Drone Actual Trajectory')
xlim([-1.0 6])
ylim([-3.0 3.0])
zlim([0.0 5])
grid on
if length(contact_msg.States) > 1
        
        
        X_contact = [X_contact, contact_msg.States(2).ContactPositions(1).X];
        Y_contact = [Y_contact, contact_msg.States(2).ContactPositions(1).Y];
        Z_contact = [Z_contact, contact_msg.States(2).ContactPositions(1).Z];      
        
        X_contact2 = [X_contact2, contact_msg.States(2).ContactPositions(2).X];
        Y_contact2 = [Y_contact2, contact_msg.States(2).ContactPositions(2).Y];
        Z_contact2 = [Z_contact2, contact_msg.States(2).ContactPositions(2).Z]; 
              
        FX_contact = [FX_contact, contact_msg.States(2).TotalWrench.Force.X];
        FY_contact = [FY_contact, contact_msg.States(2).TotalWrench.Force.Y];
        FZ_contact = [FZ_contact, contact_msg.States(2).TotalWrench.Force.Z];
        
        F_norm=[F_norm,norm([contact_msg.States(2).TotalWrench.Force.X,contact_msg.States(2).TotalWrench.Force.Y,contact_msg.States(2).TotalWrench.Force.Z])];
        
        figure(f2);
        plot3(X_contact,Y_contact,Z_contact,'r',X_contact2,Y_contact2,Z_contact2,'b','LineWidth',2)
        axis equal
        xlabel('x[m]')
        ylabel('y[m]')
        zlabel('z[m]')
        title('Drone contact points')
        xlim([3.5 5])
        ylim([-2.0 2.0])
        zlim([0.0 1.5])
        grid on
        
        %{
        figure(f3);
        plot3(FX_contact,FY_contact,FZ_contact,'ro')
        axis equal
        xlabel('x[m]')
        ylabel('y[m]')
        zlabel('z[m]')
        title('Drone contact Forces')
        xlim([-50.0 50.0])
        ylim([-50.0 50.0])
        zlim([-50.0 50.0])
        grid on
        %}
        figure(f3);
        plot(X_contact,F_norm,'g','LineWidth',2)
        axis equal
        xlabel('x[m]')
        ylabel('F[N]')
        ylim([0.0 5.0])
        title('Drone contact Force')
        grid on
        
end
%{
if length(contact_msg.States)  == 1
        X_contact = [X_contact, contact_msg.States(1).ContactPositions(1).X];
        Y_contact = [Y_contact, contact_msg.States(1).ContactPositions(1).Y];
        Z_contact = [Z_contact, contact_msg.States(1).ContactPositions(1).Z]; 
        
        FX_contact = [FX_contact, contact_msg.States(1).TotalWrench.Force.X];
        FY_contact = [FY_contact, contact_msg.States(1).TotalWrench.Force.Y];
        FZ_contact = [FZ_contact, contact_msg.States(1).TotalWrench.Force.Z]; 
end 
%}

%hold on
%plot3(X,Y,Z,'ko','LineWidth',2)
end
%rosshutdown;
function PoseCallback(msg)    
end
