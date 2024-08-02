# System
BaselineManeuverSystem

# DomainNatLang
All maneuver thread missions in which the wind parameters are all set to 0.

# Domain
```code
(config["environment"]["weather_parameter"]["wind_NED_meters_per_second"]["0"]==0
and config["environment"]["weather_parameter"]["wind_NED_meters_per_second"]["1"]==0
and config["environment"]["weather_parameter"]["wind_NED_meters_per_second"]["2"]==0 ) 
```

# ClaimNatLang
If the drone does not start in any specified keep out zone, it will avoid all specified keep out
zones for the entirety of the mission. 

# Claim
```code
# not in any of the keep out zones at intial time (T=0) implies not in any of the keep out zones in the future
(ego.T==0 and not any([(z.region in ego.position) for z in keep_out_zones]))
implies always not any([(z.region in ego.position) for z in keep_out_zones])
```

# Probability
1.0