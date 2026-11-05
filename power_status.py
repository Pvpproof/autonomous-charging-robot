"""Read-only driver voltage. No voltage-to-SOC conversion or battery model."""
import math

SOURCE = '/PowerVoltage'
STALE_SECONDS = 3.0
# The inspected driver decodes a signed 16-bit millivolt field.
MAX_DRIVER_VOLTAGE = 32.767


class PowerStatus:
    source = SOURCE
    maximum = MAX_DRIVER_VOLTAGE

    def __init__(self):
        self.value = None
        self.received_at = None
        self.invalid = False

    def update(self, value, now):
        self.received_at = now
        self.invalid = not math.isfinite(value) or not 0 < value <= self.maximum
        self.value = None if self.invalid else value

    def snapshot(self, now):
        age = None if self.received_at is None else now-self.received_at
        status = 'missing' if age is None else 'invalid' if self.invalid else 'stale' if not 0 <= age <= STALE_SECONDS else 'fresh'
        return dict(source=self.source,percentage=None,voltage=self.value if status=='fresh' else None,
                    age=age,status=status)


class Power48Status(PowerStatus):
    # Upstream checks only response length. Fresh means received, not verified.
    source = '/PowerVoltage48V'
    maximum = 655.35  # Unsigned two-byte register divided by 100, not a safe operating range.


def power_text(data, power48=None):
    status=data.get('status','missing')
    voltage=data.get('voltage')
    if status=='fresh' and voltage is not None:
        detail='底盘供电：{:.2f} V · {:.1f}秒前'.format(voltage,data['age'])
    else:
        detail='底盘供电：不可用（'+{'missing':'等待数据','invalid':'驱动数据无效','stale':'数据已过期'}.get(status,'未确认')+'）'
    raw=power48 or {}
    if raw.get('status')=='fresh' and raw.get('voltage') is not None:
        extra='48V接口（未校验）：{:.2f} V · {:.1f}秒前'.format(raw['voltage'],raw['age'])
    else:
        reason={'missing':'等待数据','invalid':'数据无效','stale':'数据已过期'}.get(raw.get('status','missing'),'未确认')
        extra='48V接口（未校验）：不可用（'+reason+'）'
    return '底盘电量：— %（无可靠百分比来源）\n'+detail+'\n'+extra
