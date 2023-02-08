# Solar Controller

## System commands

### 1. Enable heating

```sh
echo -n '{"command": "set-parameter", "args": { "name": "heating_enabled", "value": 1}}' | pub -broker ssl://mqtt.broker.name:443 -topic 'solar-controller/command' -username solar-controller -password secret -qos 1
```

## License

MIT
