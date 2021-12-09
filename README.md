# XsensSampleController

Sample controller to demonstrate how to use XSens sensors with `mc_rtc`

Dependencies:
- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)
- [mc_xsens_plugin](https://github.com/arntanguy/mc_xsens_plugin)

## Usage

Add to your `~/.config/mc_rtc/mc_rtc.yaml` configuration:

```yaml
Enabled: XsensSampleController
```

Make sure to run the MVN Analyze Pro software and check the streaming address in `XsensPlugin.yaml`.