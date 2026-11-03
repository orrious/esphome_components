# custom_components
My collection of components for esphome

To use these components, use the [external components](https://esphome.io/components/external_components.html).

The WP-CB01 dual-control-box bridge is in [`components/jsdrive2`](components/jsdrive2).
The original single-box `jsdrive` component remains in [`components/jsdrive`](components/jsdrive).
See [`wp-cb01-dual.yaml`](wp-cb01-dual.yaml) for the ESPHome configuration.

Example:
```yaml
external_components:
  - source:
      type: git
      url: https://github.com/ssieb/custom_components
    components: [ keypad ]
```

