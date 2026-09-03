# Contributing

Contributions are welcome when they keep the mathematics transparent and the experiments reproducible.

## Development setup

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e .
python -m unittest discover -s tests -v
ekf-demo
```

## Change requirements

- Add tests for numerical behavior, input validation, and new models.
- Keep simulations seeded or otherwise deterministic.
- Document state ordering, units, and covariance assumptions.
- Avoid coupling the filter core to middleware or a specific robot.
- Use synthetic measurements only; do not commit operational sensor data.

Pull requests should explain the model change, its assumptions, and the evidence used to verify it.
