# ১০০টি রিয়েলিস্টিক ড্রাইভিং সিনারিও জেনারেটর (MATLAB)

## প্রয়োজনীয়তা
- MATLAB
- **Automated Driving Toolbox** (অবশ্যই লাগবে; আপনার Apps লিস্টে `Driving Scenario Designer`, `RoadRunner` ইত্যাদি থাকা মানে এটা ইনস্টল করা আছে)

## ফাইলগুলো কী কাজ করে

| ফাইল | কাজ |
|---|---|
| `complexityLibrary.m` | সেফ ও রিস্কি আচরণের ১৫টি "মডিউল" এর তালিকা (aggressive lane change, child chasing ball, jaywalking, sudden braking ইত্যাদি) — প্রতিটির লেবেল (0=safe, 1=risky) সহ |
| `createRoadNetwork.m` | হাইওয়ে / গ্রামীণ রাস্তা / গ্রামের রাস্তা / মিক্সড (হাইওয়ে→রুরাল→ভিলেজ) রোড নেটওয়ার্ক তৈরি করে |
| `generateScenario.m` | একটা সম্পূর্ণ সিনারিও বানায়: এলোমেলোভাবে রোড টাইপ বেছে নেয়, Ego vehicle বসায়, ৫-৬টা কমপ্লেক্সিটি ইভেন্ট বেছে সেগুলোর জন্য actor (car/truck/bus/bicycle/pedestrian/child) ও trajectory তৈরি করে |
| `runAndRecordScenario.m` | সিনারিও পুরোটা সিমুলেট করে, প্রতিটা টাইমস্টেপে প্রতিটা actor-এর position/velocity/yaw লগ করে, এবং `.mat` + `.csv` আকারে সেভ করে |
| `visualizeScenario.m` | যেকোনো একটা সিনারিও visually প্লেব্যাক করে দেখায় (bird's-eye view) |
| `main_generate_100_scenarios.m` | মূল স্ক্রিপ্ট — এটা রান করলে ১০০টা সিনারিও অটোমেটিক জেনারেট, সিমুলেট ও সেভ হয়ে যাবে |

## কীভাবে রান করবেন

1. এই ৬টা `.m` ফাইল একই ফোল্ডারে রাখুন এবং MATLAB-এ সেই ফোল্ডারকে Current Folder বানান।
2. রান করুন:
   ```matlab
   main_generate_100_scenarios
   ```
3. শেষ হলে `dataset_100_scenarios/` ফোল্ডারে পাবেন:
   - `scenario_001.mat` ... `scenario_100.mat` — প্রতিটা সিনারিও অবজেক্ট + ground truth টেবিল
   - `scenario_XXX_trajectories.csv` — প্রতি সিনারিওর ফ্রেম-বাই-ফ্রেম position/velocity ডেটা (এটাই আপনার ML feature-এর মূল সোর্স)
   - `scenario_XXX_labels.csv` — কোন actor safe আর কোনটা risky
   - `dataset_summary.csv` — ১০০ সিনারিওর ওভারভিউ (রোড টাইপ, ইভেন্ট সংখ্যা, safe/risky ব্রেকডাউন)

## একটা সিনারিও visually দেখতে চাইলে

```matlab
[scenario, gtTable, meta] = generateScenario(5, 1005);
visualizeScenario(scenario);
```

আরও রিচ 3D ভিউয়ের জন্য App-এও খুলতে পারবেন:
```matlab
drivingScenarioDesigner(scenario);
```

## Classification Learner-এ ডেটা কীভাবে ব্যবহার করবেন

1. সব `_trajectories.csv` ফাইল একসাথে লোড করে প্রতিটা actor-এর জন্য feature বের করুন — যেমন average speed, max lateral acceleration, min distance to ego, sudden speed-change rate ইত্যাদি (per-actor সামারি, প্রতি সিনারিওর প্রতিটা actor থেকে একটা row)।
2. `_labels.csv` থেকে সেই actor-এর label (0/1) জয়েন করুন।
3. এই ফাইনাল ফিচার টেবিলটা `Classification Learner` App-এ ইম্পোর্ট করুন এবং ট্রেইন করুন।

## নোট / কাস্টমাইজেশন

- প্রতিটা রোডের coordinate (`createRoadNetwork.m`-এ) illustrative — আপনি চাইলে Driving Scenario Designer App-এ ইম্পোর্ট করে ভিজুয়ালি রোডের বাঁক, curve, lane সংখ্যা এডিট করতে পারবেন।
- `complexityLibrary.m`-এ নতুন কমপ্লেক্সিটি ইভেন্ট যোগ করতে চাইলে সেখানে নতুন এন্ট্রি বসিয়ে দিন এবং `generateScenario.m`-এর `generateManeuverWaypoints` ও `generateSpeedProfile` ফাংশনে সেই নামের একটা `case` যোগ করুন।
- `numScenarios` বা প্রতি সিনারিওর `numEvents` রেঞ্জ (`randi([5 6])`) চাইলে বদলে নিতে পারবেন।
