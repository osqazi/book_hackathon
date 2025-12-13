---
title: طبیعیات کے اصول
sidebar_position: 2
description: انسان نما روبوٹ سمولیشن میں gravity، inertia، friction، اور collisions کو سمجھیں
---

# انسان نما سمولیشن کے لیے طبیعیات کے اصول

## تعارف

انسان نما روبوٹ simulate کرنا wheeled روبوٹ یا stationary بازو simulate کرنے جیسا نہیں ہے۔ انسان نما کو دو ٹانگوں پر balance برقرار رکھنا چاہیے، گرے بغیر متحرک حرکات manage کرنا چاہیے، اور contact کے ذریعے ماحول کے ساتھ interact کرنا چاہیے۔ یہ چیلنجز طبیعیات کو سمجھنے کی ضرورت رکھتے ہیں جو دو پیروں پر چلنے اور manipulation کو govern کرتی ہے۔

یہ حصہ چار اہم طبیعیاتی اصولوں کو تلاش کرتا ہے جو انسان نما سمولیشن کو منفرد بناتے ہیں:

1. **Gravity**: مسلسل قوت جو روبوٹ کو نیچے کھینچتی ہے
2. **Inertia**: mass کی تقسیم کی بنیاد پر حرکت میں تبدیلیوں کی مزاحمت
3. **Friction**: contact points پر قوتیں (پاؤں، ہاتھ، اشیاء)
4. **Collisions**: اثرات کا پتہ لگانا اور ان کا جواب دینا

## Gravity: مسلسل چیلنج

### Gravity کیا کرتی ہے

Gravity روبوٹ کے ہر link پر مسلسل نیچے کی قوت apply کرتی ہے:

```
F_gravity = m * g
```

جہاں:
- `m` = link کا mass (kg)
- `g` = gravitational acceleration (زمین پر 9.81 m/s²)

کل mass 50 kg والے انسان نما کے لیے:

```
F_gravity = 50 kg * 9.81 m/s² = 490.5 Newtons نیچے کی طرف
```

یہ قوت کبھی نہیں رکتی—یہاں تک کہ جب روبوٹ ساکن ہو، اسے سیدھا رہنے کے لیے gravity کی مسلسل مزاحمت کرنی چاہیے۔

### Center of Mass (CoM) اور Balance

ایک روبوٹ balance میں ہوتا ہے جب اس کے **Center of Mass (CoM)** کا projection اس کے **support polygon** (زمین کے ساتھ contact points سے بندھے ہوئے علاقے) کے اندر آتا ہے۔

دو پیروں والے انسان نما کے لیے:

**دونوں پاؤں پر کھڑے ہونا**:
```
Support Polygon = دونوں پاؤں کے درمیان علاقہ
CoM کو اس rectangle کے اندر project ہونا چاہیے
```

**چلنا (Single Support Phase)**:
```
Support Polygon = ایک پاؤں کا علاقہ
CoM کو اس چھوٹے علاقے کے اندر project ہونا چاہیے (balance کرنا مشکل!)
```

## Inertia: حرکت میں تبدیلیوں کی مزاحمت

### Inertia کیا ہے؟

**Inertia** حرکت میں تبدیلیوں کی مزاحمت ہے۔ یہ منحصر ہے:

1. **Mass**: بھاری اشیاء acceleration کی زیادہ مزاحمت کرتی ہیں
2. **Mass کی تقسیم**: گردش کے center کے گرد mass کیسے تقسیم ہے

Rotational motion کے لیے، ہم **inertia tensor** استعمال کرتے ہیں:

```xml
<inertial>
  <mass value="2.0"/>  <!-- kg -->
  <inertia ixx="0.05" ixy="0.0" ixz="0.0"
           iyy="0.05" iyz="0.0" izz="0.01"/>
</inertial>
```

- **ixx, iyy, izz**: X، Y، Z axes کے گرد moments of inertia
- **ixy, ixz, iyz**: Products of inertia (symmetric objects کے لیے عام طور پر zero)

## Friction: Contact کا چیلنج

### Friction کی اقسام

Friction contact points پر sliding motion کی مزاحمت کرتی ہے۔ انسان نما کے لیے، یہ متاثر کرتی ہے:

- **پاؤں-زمین contact**: بغیر پھسلے چلنا
- **ہاتھ-چیز contact**: گرائے بغیر پکڑنا
- **Joint friction**: motors میں اندرونی مزاحمت (عام طور پر علیحدہ model کی جاتی ہے)

### Coulomb Friction Model

زیادہ تر simulators **Coulomb friction model** استعمال کرتے ہیں:

```
F_friction_max = μ * F_normal
```

جہاں:
- `μ` = coefficient of friction (مادی خاصیت)
- `F_normal` = normal force (surface کے perpendicular)

### Gazebo میں Friction کو configure کرنا

SDF/URDF collision elements میں:

```xml
<collision name="foot_collision">
  <geometry>
    <box size="0.1 0.05 0.02"/>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>      <!-- پہلی friction direction -->
        <mu2>1.0</mu2>    <!-- دوسری friction direction -->
      </ode>
    </friction>
  </surface>
</collision>
```

## Collisions: Detection اور Response

### Collisions کیا ہیں؟

ایک **collision** اس وقت ہوتا ہے جب دو اشیاء کی geometries آپس میں intersect ہوتی ہیں۔ Simulators کو چاہیے:

1. **Detect** کریں جب collision ہو (computationally مہنگا)
2. **Contact points compute** کریں (جہاں surfaces چھوتی ہیں)
3. **Contact forces apply** کریں penetration کو روکنے کے لیے
4. **Dynamics resolve** کریں (bounce، slide، یا stick together)

### Collision Geometry بمقابلہ Visual Geometry

URDF visual اور collision meshes کو الگ کرتا ہے:

```xml
<link name="arm">
  <visual>
    <geometry>
      <mesh filename="arm_detailed.stl"/>  <!-- دیکھنے کے لیے high-poly -->
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.04" length="0.3"/>  <!-- رفتار کے لیے سادہ -->
    </geometry>
  </collision>
</link>
```

**الگ کیوں؟**
- **Visual**: حقیقی rendering کے لیے تفصیلی mesh (collisions test کرنے میں سست)
- **Collision**: تیز طبیعیات کے لیے simplified shape (primitives جیسے boxes، cylinders، spheres)

## خلاصہ

انسان نما کے لیے طبیعیات سمولیشن کو سمجھنے کی ضرورت ہے:

- **Gravity**: مسلسل نیچے کی قوت جو فعال balance کنٹرول کی ضرورت رکھتی ہے (support polygon کے اندر CoM)
- **Inertia**: Mass کی تقسیم rotational dynamics کو متاثر کرتی ہے (درست inertia tensors specify کریں)
- **Friction**: Contact forces چلنے اور پکڑنے کو قابل بناتی ہیں (پاؤں کے لیے μ = 0.8–1.2)
- **Collisions**: performance کے لیے simplified geometry کے ساتھ detection اور response

**اہم نکات**:
- انسان نما فطری طور پر غیر مستحکم ہیں—سمولیشن balance کے مسائل جلد ظاہر کرتی ہے
- درست URDF parameters (mass، inertia، friction) حقیقی رویے کے لیے اہم ہیں
- Simplified collision geometry performance اور درستگی میں balance کرتی ہے
- Parameters tune کرنے کے لیے حقیقی ہارڈ ویئر کے خلاف سمولیشن validate کریں

---

**جاری رکھیں**: [سینسر سمولیشن](./sensors.md)

## حوالہ جات

Open Robotics. (2024). *Gazebo Documentation: Physics Engines*. https://gazebosim.org/api/sim/7/physicsplugin.html
