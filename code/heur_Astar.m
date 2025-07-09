function y = heur_Astar(Start,Target,Position)

g = sqrt( (Start(1) - Position(1) )^2 + (Start(2) - Position(2))^2 );
h = sqrt( (Target(1) - Position(1) )^2 + (Target(2) - Position(2))^2 );
f = g+h;

y = [f;g;h];
end