%All angles in radians, units in mkgs

%Import reference results from Solidworks Fluid Simulation:
%Linspace of Initial Simulation
RefVelocity1=linspace(20,10,8);
RefMisalignmentAngle1=linspace(-0.1,0.1,30)';
%Linspace of Second Simulation
RefVelocity2=linspace(20,10,3);
RefMisalignmentAngle2=linspace(-0.1,0.1,15)';
%Interpolate 2nd simulation to linspace of first simulation, reshape results of both sims to matrix 
RefForceY=interp2(RefVelocity2, RefMisalignmentAngle2, reshape(ForceVecStudy{3,:},[15,3]),RefVelocity1,RefMisalignmentAngle1,'spline');
RefForceX=interp2(RefVelocity2, RefMisalignmentAngle2, reshape(ForceVecStudy{4,:},[15,3]),RefVelocity1,RefMisalignmentAngle1,'spline');
RefAngularAcceleration=reshape(-ParametricStudy{4,:},[length(RefMisalignmentAngle1),length(RefVelocity1)]); %Subtract results because the angular reference frame of the solidworks sim is reverse of the matlab sim

%Starting conditions
MisalignmentAngle=0.0; %Angle between dart velocity vector and dart longitudinal axis
Angle=0.437; %Angle of longitudinal axis of dart with respect to ground
AngularVelocity=0;
AngularAcceleration=0;
Position=[0,0.7];
VelocityMagnitude=20;
Gravity=[0,-9.8];
Velocity=[cos(Angle-MisalignmentAngle)*VelocityMagnitude, sin(Angle-MisalignmentAngle)*VelocityMagnitude];
Mass=.22;
Timestep=0.0005;
Endtime=1.7;

%Initialize loop variables
j=1;
PlotPosition=zeros(length(0:Timestep:Endtime),2);
PlotMisalignmentAngle=zeros(length(0:Timestep:Endtime),1);
PlotNormForce=zeros(length(0:Timestep:Endtime),1);

for t=0:Timestep:Endtime

    %Add iteration variables to plotting vectors
    PlotPosition(j,:)=Position;
    PlotNormForce(j)=norm([ForceX,ForceY]);
    PlotMisalignmentAngle(j)=MisalignmentAngle;

    %Simulation
    ForceX=interp2(RefVelocity1,RefMisalignmentAngle1,RefForceX,norm(Velocity),MisalignmentAngle,'spline'); %Interpolate the query point into your reference data set
    ForceY=interp2(RefVelocity1,RefMisalignmentAngle1,RefForceY,norm(Velocity),MisalignmentAngle,'spline');
    AngularAcceleration=interp2(RefVelocity1,RefMisalignmentAngle1,RefAngularAcceleration,norm(Velocity),MisalignmentAngle,'spline');
    Acceleration=Gravity+([ForceX,ForceY]./Mass);
    Position=Position+Velocity.*Timestep+0.5.*Acceleration.*Timestep^2;
    Velocity=Velocity+Acceleration*Timestep;
    Angle=Angle+AngularVelocity*Timestep+0.5*AngularAcceleration*Timestep^2;
    AngularVelocity=AngularVelocity+AngularAcceleration*Timestep;
    MisalignmentAngle=Angle-atan(Velocity(2)/Velocity(1));

    j=j+1; %increment iteration var

end

%Plot figures:

figure
plot(PlotPosition(:,1),PlotPosition(:,2))
xlabel('X(meters)') 
ylabel('Y(meters)') 
figure
plot(0:Timestep:Endtime,PlotMisalignmentAngle)
xlabel('Time (s)') 
ylabel('Misalignment Angle (rad)') 
figure
plot(0:Timestep:Endtime,PlotNormForce)
xlabel('Time (s)') 
ylabel('Aerodynamic Force Magnitude (N)') 

