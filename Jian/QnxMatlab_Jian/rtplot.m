btnXLg = 85; %100; % button length;
btnYSp = 70; % vertical spacing of buttons
btnYHt = 50; % button height

scrsz = get(0,'ScreenSize');
% [ horizontal position from the left, vertical position from the bottom, width of the figure, height of the figure]
f = figure('Name','Qnx Kit Real Time Plot','Position',...
    [round(0.35*scrsz(3)), 300, round(0.6*scrsz(3)) round(0.6*scrsz(4))]); 


% push buttons:

oneshot = 0;

uicontrol('Style', 'pushbutton', 'String', 'Start',...
     'Position', [5 5+1*btnYSp btnXLg btnYHt], 'Callback', 'rtplot_start');
%uicontrol('Style', 'pushbutton', 'String', 'Start',...
%    'Position', [5 5+1*btnYSp btnXLg btnYHt], 'Callback', 'rtplot_start_dataOnly');
uicontrol('Style', 'pushbutton', 'String', 'Stop',...
    'Position', [5 5+0*btnYSp btnXLg btnYHt], 'Callback', 'rtplot_stop');
oneshot_checkbox=uicontrol('Style','checkbox','String','One Shot',...
    'BackgroundColor',[.8 .8 .8],...
    'Position', [5 5+2*btnYSp btnXLg btnYHt],'CallBack',... 
   ['oneshot=get(oneshot_checkbox,''value'');']);


