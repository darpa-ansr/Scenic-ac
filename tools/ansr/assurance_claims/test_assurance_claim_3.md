# DomainNatLang
Perfect visibility (does not account for time of day).

# Domain
```
config["environment"]["weather_parameter"]["fog"] == 0 and
config["environment"]["weather_parameter"]["rain"] == 0 and
config["environment"]["weather_parameter"]["snow"] == 0 and
config["environment"]["weather_parameter"]["dust"] == 0 and
config["environment"]["weather_parameter"]["foliage"] == 0
```

# ClaimNatLang
The position of car000 is reported to within two widths of ground truth position at least once
during the mission.

# Claim
```
eventually any([distance from targets_groundtruth["car000"] to q.position <= 2*p.width
        for q in targets_reported if q.id == "car000"
    ])
```