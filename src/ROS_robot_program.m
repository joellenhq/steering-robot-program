ip = "http://192.168.204.128:11311";
rosinit(ip);

%Opracowanie mapy
subs1 = rossubscriber('/odom')
subs2 = rossubscriber('/scan') 
pub1 = rospublisher('/cmd_vel','geometry_msgs/Twist') 
pause(1);

odom_data = receive(subs1,10);
pose_orientation = odom_data.Pose.Pose.Orientation;
pose_position = odom_data.Pose.Pose.Position;
odom_data1=quat2eul([pose_orientation.W pose_orientation.X pose_orientation.Y pose_orientation.Z]); 

theta=odom_data1(1);
x=pose_position.X;
y=pose_position.Y;
start_pose=[x y]; 
start_position=[x y theta]; 

points1=[x y; x+2 y;x+2 y+3;x-3 y+3;x-1 y]; 
points2=[x y; x+1.5 y; x+1.5 y+2.5; x y+2.5; x y+0.5; x-1 y+0.5; x-1 y+3];


controller=controllerPurePursuit
controller.Waypoints=points1;
controller.DesiredLinearVelocity = 0.1; 
controller.MaxAngularVelocity = 2.0; 
controller.LookaheadDistance = 0.5; 

sample_Time=0.2;
rate=rateControl(1/sample_Time);
end_pose=[x y-1]; 
end_position=[x y-1 theta]; 
tolerancja=0.1; 

msg1 = rosmessage(pub1); 

roznica=norm(start_pose-end_pose); 
positions=[];
current_pose=[];
angle1=0;
sterowanie_p=0;
roznica=norm(start_pose-end_pose); 
positions=[];
current_pose=[];
scanData=[];

maxLidarRange = 3;
mapResolution = 50;
slamAlg=lidarSLAM(mapResolution,maxLidarRange);
slamAlg.LoopClosureThreshold=210;
slamAlg.LoopClosureSearchRadius=8;

while abs(roznica)>=tolerancja 
    odom_data = receive(subs1,10); 
    pose_orientation1= odom_data.Pose.Pose.Orientation;
    pose_position1 = odom_data.Pose.Pose.Position;
    odom_data2=quat2eul([pose_orientation1.W pose_orientation1.X pose_orientation1.Y pose_orientation1.Z]); 
    
    theta1=odom_data2(1);
    x1=pose_position1.X;
    y1=pose_position1.Y;
    current_pose=[x1 y1];
    current_position=[x1 y1 theta1]; 
    roznica=norm(current_pose-end_pose) 
    
    scan_data=receive(subs2,10);
    ranges=double(scan_data.Ranges);
    odom_data4=double(scan_data.readScanAngles);
    
    scan=lidarScan(ranges,odom_data4);
    
    [v,omega] = controller(current_position); 
    positions=cat(2,positions,current_position); 
    scanData=cat(2,scanData,scan);
    
    msg1.Linear.X = v; 
    msg1.Angular.Z = omega;
  
    pub1.send(msg1);
    
    slamAlg=lidarSLAM(mapResolution,maxLidarRange);
    close all
    figure(1);
    for i=1:2:length(scanData)
        [isScanAccepted, loopClosureInfo]=slamAlg.addScan(scanData(i));
        if isScanAccepted
            fprintf('Added scan %d \n',i);
        end
        if mod(i,40)==1
            show(slamAlg);
            drawnow
        end
    end
    figure(2);
    show(slamAlg);
    title({'Map of the Environment'});

    [scans,optimizedPoses]=scansAndPoses(slamAlg);
    map=buildMap(scans,optimizedPoses,mapResolution,maxLidarRange);

    waitfor(rate); 
end

msg1.Linear.X = 0;
msg1.Angular.Z = 0;
pub1.send(msg1);

figure(3);
show(map);
save('Map.mat','map');

hold on
show(slamAlg.PoseGraph,'IDs','off');
hold off

title('Occupancy grid map build using lidar SLAM');
mapMatrix=map.occupancyMatrix();
mapMatrix(mapMatrix<=0.5) = 0;
mapMatrix(mapMatrix>0.5) = 1;
mapbin = binaryOccupancyMap(mapMatrix);
figure(4);
show(mapbin);


plot(positions(1,:),positions(2,:),points(:,1),points(:,2),'k--d');
xlabel("x");
ylabel("y");
rosshutdown

%Lokalizowanie robota na utworzonej mapie
%load('mapa.mat');
show(map)
%rosinit("http://192.168.172.128:11311")
subs1 = rossubscriber('/odom')
subs2 = rossubscriber('/scan') 
%subsPaczki = rossubscriber('/paczki') 
pub1 = rospublisher('/cmd_vel','geometry_msgs/Twist') 
pause(1);
odom_data = receive(subs1,10);
pose_orientation = odom_data.Pose.Pose.Orientation;
pose_position = odom_data.Pose.Pose.Position;
odom_data1=quat2eul([pose_orientation.W pose_orientation.X pose_orientation.Y pose_orientation.Z]); 
theta=odom_data1(1);
x=pose_position.X;
y=pose_position.Y;
start_pose=[x y]; 
start_position=[x y theta]; 

sample_Time=0.7;
rate=rateControl(1/sample_Time);

msg1 = rosmessage(pub1); 

odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits=[0.45 4];
rangeFinderModel.Map = map;

tftree=rostf;
waitForTransform(tftree,'/base_footprint','/base_footprint');
sensorTransform = getTransform(tftree,'/base_footprint','/base_footprint');
sensor_R=sensorTransform.Transform.Rotation;
sensor_T=sensorTransform.Transform.Translation;
laserQuat = [sensor_R.W sensor_R.X sensor_R.Y sensor_R.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

rangeFinderModel.SensorPose = [sensor_T.X sensor_T.Y laserRotation(1)];

amcl=monteCarloLocalization;
amcl.UseLidarScan = true;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
amcl.UpdateThresholds=[0.2,0.2,0.2];
amcl.ResamplingInterval=1;
amcl.ParticleLimits=[500 5000];
amcl.GlobalLocalization = true;

visualizationHelper=ExampleHelperAMCLVisualization(map);


hasRotated=0;
angle1=0;
sterowanie_p=0;

current_pose=[];
scanData=[];

i=0;
while hasRotated==0
    odom_data = receive(subs1,10); 
    pose_orientation1= odom_data.Pose.Pose.Orientation;
    pose_position1 = odom_data.Pose.Pose.Position;
    odom_data2=quat2eul([pose_orientation1.W pose_orientation1.X pose_orientation1.Y pose_orientation1.Z]); 
    
    theta1=odom_data2(1);
    x1=pose_position1.X;
    y1=pose_position1.Y;
    current_pose=[x1 y1];
    current_position=[x1 y1 theta1]; 

    scan_data=receive(subs2,10);
    ranges=double(scan_data.Ranges);
    angles=double(scan_data.readScanAngles);
    scan=lidarScan(ranges,angles);
    
    
    [isUpdated,estimatedPose,estimatedCovariance]=amcl(current_position,scan);
    
    msg1.Linear.X=0.1;
    pub1.send(msg1);
    
    if isUpdated
        i=i+1;
        plotStep(visualizationHelper,amcl,estimatedPose,scan,i)
        if i>=10
            hasRotated=1;
            
            save('Localization.mat','map');
        end
    end
    waitfor(rate);    
end

msg1.Linear.X = 0;
pub1.send(msg1);



%clear all
load('mapa.mat');
load('Localization.mat');
show(map)

%Planowanie trajektorii
occMapInf = vehicleCostmap(map);
occMapInf.CollisionChecker = inflationCollisionChecker("InflationRadius",0.12);
plot(occMapInf)
subs1 = rossubscriber('/odom')
odom_data = receive(subs1,10);
pose_orientation = odom_data.Pose.Pose.Orientation;
pose_position = odom_data.Pose.Pose.Position;
odom_data1=quat2eul([pose_orientation.W pose_orientation.X pose_orientation.Y pose_orientation.Z]); 
%Miejsca postojów
theta=odom_data1(1);
x=pose_position.X;
y=pose_position.Y;
start_pose1=[x, y]; 
start1=[x, y, theta]; 

box_pose1=[-0.5 4]; 
box1=[box_pose1 -0.5*pi]; 
box_pose2=[-0.5 7]; 
box2=[box_pose2 -0.5*pi]; 
box_pose3=[-0.5 -10]; 
box3=[box_pose3 -0.5*pi]; 

goal_pose1=[12 10]; 
goal1=[goal_pose1 0.5*pi]; 
goal_pose2=[12 7]; 
goal2=[goal_pose2 0.5*pi]; 
goal_pose3=[12 5]; 
goal3=[goal_pose3 0.5*pi]; 


sample_Time=0.7;
rate=rateControl(1/sample_Time);
bounds=[map.XWorldLimits;map.YWorldLimits; [-pi,pi]];
ss=stateSpaceDubins(bounds);
ss.MinTurningRadius = 1.0;

paczka=1%checkPackage

while(true)
    points=[]
    if paczka>0
        %obecna pozycja
        odom_data = receive(subs1,10);
        pose_orientation = odom_data.Pose.Pose.Orientation;
        pose_position = odom_data.Pose.Pose.Position;
        odom_data1=quat2eul([pose_orientation.W pose_orientation.X pose_orientation.Y pose_orientation.Z]); 
        theta=odom_data1(1);
        x=pose_position.X;
        y=pose_position.Y;
        current_pose=[x, y]; 
        current=[x, y, theta]; 
        if paczka==1
            target1=box1;
            target2=goal1;
        elseif paczka==2
            target1=box2;
            target2=goal2;
        elseif paczka==3
            target1=box3;
            target2=goal3;
        else 
            target1=start;
        end
        plot(current(1), current(2),'ro');
        plot(target1(1), target1(2), 'mo');
        hold on
        r = 0.5;
        plot([current(1),current(1)+r*cos(current(3))],[current(2)+r*sin(current(3))],'-r');
        plot([target1(1),target1(1)+r*cos(target1(3))],[target1(2),target1(2)+r*sin(target1(3))],'m-')
        hold off

        bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

        stateValidator=validatorVehicleCostmap(ss);
        stateValidator.Map = occMapInf;
        stateValidator.ValidationDistance = 0.05;

        planner = plannerRRTStar(ss, stateValidator);
        planner.MaxConnectionDistance = 0.6;
        planner.MaxIterations = 30000;
        planner.GoalReachedFcn = @goalReached;

        rng(0,'twister');

        [pthObj, solnInfo] = plan(planner, current, target1);

        plot(occMapInf)
        hold on

        plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-');

        plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2);

        plot(start(1),start(2),'ro');
        plot(goal(1),goal(2),'mo');

        hold off

        States = pthObj.States;
        points=pthObj.States;
        save('PlannedPath.mat','States');
        %points=???
        if paczka<4 && paczka>0
            plot(target1(1), target1(2),'ro');
            plot(target2(1), target2(2), 'mo');
            hold on
            r = 0.5;
            plot([target1(1),target1(1)+r*cos(target1(3))],[target1(2)+r*sin(target1(3))],'-r');
            plot([target2(1),target2(1)+r*cos(target2(3))],[target2(2),target2(2)+r*sin(target2(3))],'m-')
            hold off

            bounds1 = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

            stateValidator1=validatorVehicleCostmap(ss);
            stateValidator1.Map = occMapInf;
            stateValidator1.ValidationDistance = 0.05;

            planner1 = plannerRRTStar(ss, stateValidator1);
            planner1.MaxConnectionDistance = 0.6;
            planner1.MaxIterations = 30000;
            planner1.GoalReachedFcn = @goalReached;

            rng(0,'twister');

            [pthObj1, solnInfo1] = plan(planner1, target1, target2);

            plot(occMapInf)
            hold on

            plot(solnInfo1.TreeData(:,1),solnInfo1.TreeData(:,2),'.-');

            plot(pthObj1.States(:,1),pthObj1.States(:,2),'r-','LineWidth',2);

            plot(target1(1),target1(2),'ro');
            plot(target2(1),target2(2),'mo');

            hold off

            States = pthObj1.States;

            save('PlannedPath1.mat','States');
            points=cat(2,points,pthObj1.States)
        end
    %ruch po zaplanowanej ścieżce
    %kilka parametrów do zmiany
    controller1=controllerPurePursuit
    controller1.Waypoints=points;
    controller1.DesiredLinearVelocity = 0.1; 
    controller1.MaxAngularVelocity =0.8; 
    controller1.LookaheadDistance = 0.7; 

    controller2=controllerVFH;
    controller2.UseLidarScan=true;
    controller2.DistanceLimits = [0.05 1]; 
    controller2.RobotRadius = 0.1;
    controller2.MinTurningRadius = 0.2;
    controller2.SafetyDistance = 0.1;

    sample_Time=0.5; 
    rate=rateControl(1/sample_Time);
    tolerancja=0.1; 

    msg1 = rosmessage(pub1); %utworzenie wiadomości
    roznica=norm(current-target2); 
    
    while abs(roznica)>=tolerancja
        odom_data = receive(subs1,10); %odczytanie aktualnych danych
        pose1= odom_data.Pose.Pose.Orientation;
        pose_position1 = odom_data.Pose.Pose.Position;
        odom_data2=quat2eul([pose1.W pose1.X pose1.Y pose1.Z]); 
        %obecna lokalizacja
        theta1=odom_data2(1);
        x1=pose_position1.X;
        y1=pose_position1.Y;
        current_pose=[x1 y1];%pozycja obecna
        current_position=[x1 y1 theta1]'; %obecna pozycja i orientacja
        roznica=norm(current_pose-target2) %sprawdzenie jak daleko robot jest od celu
        positions=cat(2,positions,current_position); %dopisanie pozycji do tablicy 
    
        scan_data=receive(subs2,10); %odczytywanie danych z czujnika Lidar
        ranges=double(scan_data.Ranges);
        odom_data4=double(scan_data.readScanAngles);
    
        scan=lidarScan(ranges,odom_data4);
        %wyliczenie nowych wartości zmiennych sterujących na podstawie obecnej
        %pozycji
        [v,omega] = controller1(current_position); 
    
        %obliczenie nowych predkosci na podstawie wartosci obliczonych przez
        %kontroler 1
    
        angle1=angle1+omega.*sample_Time; %obliczenie potrzebnego kata
        sterowanie=controller2(scan,angle1); %obliczenie bezkolizyjnej orientacji
        if ~isnan(sterowanie) %sprawdzenie czy wartosc nie jest numeryczna
            new_omega=(sterowanie-sterowanie_p)./sample_Time; %obliczenie nowej predkosci katowej
            sterowanie_p=sterowanie; %zapisanie poprzedniej wartosci sterowania
            new_v=v;
        else
            %ustawienie predkosci liniowej na 0
            new_v=0.0;
            if sterowanie_p>=0
            %ustawienie predkosci katowej na wartosc stala
                new_omega=0.5
            else
            %w drugim przypadku ustawienie predkosci katowej w przeciwnym
            %kierunku
                new_omega=-0.5
            end
        end
    
        msg1.Linear.X = new_v; %ustawienie predkosci
        msg1.Angular.Z = new_omega;
        %wyslanie wiadomosci z nowymi wartosciami predkosci
        pub1.send(msg1);
    
        waitfor(rate);  %ustawienie wykonywania petli na zadany czas próbkowania
    
    end
    paczka=2;
    end
end

rosshutdown