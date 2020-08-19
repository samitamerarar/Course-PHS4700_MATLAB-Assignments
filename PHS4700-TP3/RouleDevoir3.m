%
% Devoir 3
% Lancer balle-boite
% 
format long
clear;
%
% Conditions initiales
% tls    temps du lancer
% vbals  vitesse initiale de la balle (m/s)
% wbois  vitesse angulaire initiale de la boite (rads/s)
%
tls=[0.66 1.1 0.66 1.1];
vbals([1:3],1)=[6.85;0.0;6.85];
wbois([1:3],1)=[0;0;0];
vbals([1:3],2)=[28;0.5;10];
wbois([1:3],2)=[0;0;0];
vbals([1:3],3)=[6.85;0.0;6.85];
wbois([1:3],3)=[0;2.3;0];
vbals([1:3],4)=[28;0.5;10];
wbois([1:3],4)=[0;2.3;0];
%
% Simulations
%
for Cas=1:4
  fprintf('\nSimulation %3d\n',Cas);
  fprintf('Temps du tir = %8.4f \n',tls(Cas));
  fprintf('vbal (m/s)   = (%8.4f,%8.4f,%8.4f)  \n',vbals(1,Cas),vbals(2,Cas),vbals(3,Cas));
  fprintf('wboi (r/s)   = (%8.4f,%8.4f,%8.4f)  \n',wbois(1,Cas),wbois(2,Cas),wbois(3,Cas));
  [Coup tf vbaf vbof wbof rbaf rbof]=Devoir3(vbals(:,Cas),wbois(:,Cas),tls(Cas));
  if Coup == 1
    fprintf('Collision boite-balle\n');
  else
    fprintf('Le balle touche le sol\n');
  end
  fprintf('La simulation prend fin au temps     %8.4f \n',tf);
  fprintf('Vitesse de la balle avant collision  $(%8.4f,%8.4f,%8.4f)$  m/s\n',vbaf(1,1),vbaf(2,1),vbaf(3,1));
  fprintf('Vitesse de la balle apres collision  $(%8.4f,%8.4f,%8.4f)$  m/s\n',vbaf(1,2),vbaf(2,2),vbaf(3,2));
  fprintf('Vitesse de la boite avant collision  $(%8.4f,%8.4f,%8.4f)$  m/s\n',vbof(1,1),vbof(2,1),vbof(3,1));
  fprintf('Vitesse de la boite apres collision  $(%8.4f,%8.4f,%8.4f)$  m/s\n',vbof(1,2),vbof(2,2),vbof(3,2));
  fprintf('Vitesse angulaire finale de la boite $(%8.4f,%8.4f,%8.4f)$  rad/s\n',wbof(1),wbof(2),wbof(3));
  fprintf('Position finale de la balle          $(%8.4f,%8.4f,%8.4f)$  m\n',rbaf(1),rbaf(2),rbaf(3));
  fprintf('Position finale de la boite          $(%8.4f,%8.4f,%8.4f)$  m\n\n',rbof(1),rbof(2),rbof(3));
  pause;
end
