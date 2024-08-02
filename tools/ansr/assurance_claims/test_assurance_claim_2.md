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
The position of at least one target specified in the mission specification is reported to within two
widths of groundtruth position at least once during the mission.

# Claim
```
eventually any([
    any([
        (p.id == q.id) and (distance from p.position to q.position <= 2*p.width)
        for q in targets_reported
    ])
    p in targets_groundtruth
])
```