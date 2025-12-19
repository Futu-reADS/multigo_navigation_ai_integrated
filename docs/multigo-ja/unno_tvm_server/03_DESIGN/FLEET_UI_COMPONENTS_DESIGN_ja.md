# フリートUIコンポーネント設計

**チーム:** Unno（フリート管理）
**日付:** 2025-12-16
**バージョン:** 1.0

## コンポーネント階層

```
App
├─ AuthProvider (React Context)
├─ ThemeProvider (Material-UI)
├─ Router
│  ├─ Dashboard
│  │  ├─ Map Component (Leaflet)
│  │  │  ├─ VehicleMarker (×N)
│  │  │  ├─ PathOverlay
│  │  │  └─ GeofenceLayer
│  │  ├─ StatisticsPanel
│  │  └─ AlertsList
│  ├─ MissionsPage
│  │  ├─ MissionList
│  │  ├─ MissionDetail (Modal)
│  │  └─ CreateMissionWizard
│  └─ ...
```

## 主要コンポーネント

### Mapコンポーネント
```typescript
<MapComponent>
  <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />
  <VehicleMarkers vehicles={vehicles} onClick={handleVehicleClick} />
  <PathOverlays missions={activeMissions} />
  <GeofencePolygons geofences={geofences} />
</MapComponent>
```

### Vehicleマーカー
```typescript
interface VehicleMarkerProps {
  vehicle: Vehicle;
  onClick: (vehicle: Vehicle) => void;
}

// Icon color based on status
const getMarkerIcon = (status: VehicleStatus) => {
  switch (status) {
    case 'idle': return greenIcon;
    case 'navigating': return blueIcon;
    case 'error': return redIcon;
  }
};
```

### Missionリスト
```typescript
<MissionList>
  {missions.map(mission => (
    <MissionCard
      key={mission.id}
      mission={mission}
      onCancel={handleCancel}
      onClick={() => setSelectedMission(mission)}
    />
  ))}
</MissionList>
```

## 状態管理（Redux Toolkit）

```typescript
// features/vehicles/vehiclesSlice.ts
const vehiclesSlice = createSlice({
  name: 'vehicles',
  initialState: {
    list: [],
    selected: null,
    loading: false,
  },
  reducers: {
    setVehicles(state, action) {
      state.list = action.payload;
    },
    updateVehicleStatus(state, action) {
      const vehicle = state.list.find(v => v.id === action.payload.id);
      if (vehicle) {
        vehicle.status = action.payload.status;
      }
    },
  },
});
```

## WebSocket統合

```typescript
// hooks/useWebSocket.ts
const useWebSocket = () => {
  const dispatch = useDispatch();

  useEffect(() => {
    const socket = io('wss://tvm-server.example.com');

    socket.on('vehicle_telemetry', (data) => {
      dispatch(updateVehicleStatus(data));
    });

    return () => socket.disconnect();
  }, []);
};
```

---
**参考文献:** FLEET_UI_ARCHITECTURE.md、FLEET_UI_REQUIREMENTS.md（109要件）
