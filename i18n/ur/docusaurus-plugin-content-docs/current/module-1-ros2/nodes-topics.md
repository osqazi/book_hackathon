---
title: Nodes اور Topics
sidebar_position: 2
description: انسان نما روبوٹ کنٹرول کے لیے publish-subscribe نمونے استعمال کرتے ہوئے ROS 2 nodes کیسے topics کے ذریعے بات چیت کرتے ہیں جانیں
---

# Nodes اور Topics: ROS 2 مواصلات کی بنیاد

## تعارف

ایک انسان نما روبوٹ کا تصور کریں جو چلتے ہوئے بیک وقت کیمرے کا ڈیٹا پروسیس کر رہا ہے، توازن برقرار رکھ رہا ہے، اور اپنے اگلے قدم کی منصوبہ بندی کر رہا ہے۔ ان میں سے ہر کام مختلف الگورتھم کی ضرورت ہے، مختلف رفتار سے چلتا ہے، اور یہاں تک کہ مختلف processors پر بھی عمل کر سکتا ہے۔ آپ یہ سب پیچیدگی کیسے مربوط کرتے ہیں؟

جواب **ROS 2 nodes** اور **topics** ہے۔ یہ architectural pattern آپ کے روبوٹ کے سافٹ ویئر کو آزاد، single-purpose processes میں توڑتا ہے جو معیاری پیغام کے چینلز کے ذریعے بات چیت کرتے ہیں۔

## ROS 2 Node کیا ہے؟

ایک **node** ایک آزاد process ہے جو ایک مخصوص computational task انجام دیتا ہے۔ Nodes کو factory میں specialized workers کے طور پر سوچیں—ہر ایک کا ایک کام ہے اور وہ اسے اچھی طرح کرتا ہے۔

### Nodes کی اہم خصوصیات

1. **Single Responsibility**: ہر node ایک کام پر توجہ مرکوز کرتا ہے
   - ایک کیمرا node تصویری ڈیٹا publish کرتا ہے
   - ایک balance controller node IMU readings پروسیس کرتا ہے
   - ایک motor driver node actuators کو کمانڈز بھیجتا ہے

2. **Process Isolation**: Nodes علیحدہ operating system processes کے طور پر چلتے ہیں
   - اگر ایک node crash ہو جائے، دوسرے چلتے رہتے ہیں
   - انفرادی components کو debug اور test کرنا آسان ہے
   - مختلف programming languages میں لکھا جا سکتا ہے (Python, C++)

## Topic کیا ہے؟

ایک **topic** ایک نامزد مواصلاتی چینل ہے جسے nodes پیغامات کا تبادلہ کرنے کے لیے استعمال کرتے ہیں۔ Topics **publish-subscribe pattern** کو implement کرتے ہیں:

- **Publishers** topic پر پیغامات بھیجتے (publish کرتے) ہیں
- **Subscribers** topic سے پیغامات وصول (subscribe) کرتے ہیں
- Publishers اور subscribers ایک دوسرے کے بارے میں نہیں جانتے—وہ صرف topic کا نام جانتے ہیں

## Code Example 1: منیمل Publisher

آئیں ایک سادہ node لکھیں جو انسان نما کے بائیں بازو کے لیے joint state ڈیٹا publish کرتا ہے۔

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ArmJointPublisher(Node):
    def __init__(self):
        super().__init__('arm_joint_publisher')

        # /arm/joint_states topic کے لیے publisher بنائیں
        self.publisher_ = self.create_publisher(
            JointState,           # پیغام کی قسم
            '/arm/joint_states',  # Topic کا نام
            10                    # QoS history depth
        )

        # 10 Hz پر publish کریں
        self.timer = self.create_timer(0.1, self.publish_joint_state)
        self.get_logger().info('Arm joint publisher started')

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_shoulder', 'left_elbow']
        msg.position = [0.5, 1.2]  # مثال کی پوزیشنز radians میں
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmJointPublisher()
    rclpy.spin(node)  # Node کو چلتے رہیں
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### اہم نکات

- **`Node` سے وراثت**: ہر ROS 2 node `Node` base class کو extend کرتا ہے
- **`create_publisher()`**: اعلان کرتا ہے کہ یہ node `/arm/joint_states` پر publish کرے گا
- **`create_timer()`**: ہر 0.1 سیکنڈ (10 Hz) میں `publish_joint_state()` کو کال کرتا ہے
- **`publish()`**: تمام subscribers کو پیغام بھیجتا ہے
- **`rclpy.spin()`**: Node کو زندہ رکھتا ہے اور callbacks پروسیس کرتا ہے

## خلاصہ

**Nodes** اور **topics** ROS 2 کی modular architecture کی بنیاد بناتے ہیں:

- **Nodes**: single responsibilities کے ساتھ آزاد processes
- **Topics**: asynchronous، publish-subscribe مواصلات کے لیے نامزد چینلز
- **Messages**: topics پر بھیجے گئے typed ڈیٹا structures
- **Decoupling**: Publishers اور subscribers ایک دوسرے کے بارے میں نہیں جانتے
- **Flexibility**: many-to-many مواصلات، hot-swappable components

انسان نما روبوٹکس کے لیے، اس کا مطلب ہے:
- سینسر nodes مسلسل ڈیٹا streams publish کرتے ہیں
- پروسیسنگ nodes subscribe کرتے، حساب لگاتے، اور نتائج publish کرتے ہیں
- Actuator nodes کمانڈز کو subscribe کرتے اور motors کو control کرتے ہیں
- تمام components بغیر tight coupling کے مل کر کام کرتے ہیں

اگلا، ہم request-response patterns اور طویل چلنے والے کاموں کے لیے **services** اور **actions** کو تلاش کریں گے۔

---

**جاری رکھیں**: [Services اور Actions](./services-actions.md)

## حوالہ جات

Open Robotics. (2024). *ROS 2 Documentation: Understanding Topics*. https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html
