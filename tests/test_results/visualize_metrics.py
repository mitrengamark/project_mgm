#!/usr/bin/env python3
"""
Tesztelési metrikák vizualizációs eszköz - T1, T2, T3 tesztek összehasonlítása.

Ez a szkript a lidar_filter projekt három fő tesztjének (T1, T2, T3) eredményeit
vizualizálja különböző grafikonokon és táblázatokon keresztül.

Tesztek:
- T1: Statikus környezet teszt (egyetlen álló objektum)
- T2: Mozgó robot teszt (változó távolságok)
- T3: Stressz teszt (több objektum, dinamikus környezet)

Generált kimenetek:
- Scan rate összehasonlítás (Hz)
- Detektálási megbízhatóság (%)
- Objektumok/scan grafikonok
- Kombinált metrikák (2x2 subplot)
- T3 objektum eloszlás részletes elemzés
- Teljesítmény radar chart
- Összefoglaló táblázat (CSV + kép)

Használat:
    python3 visualize_metrics.py

Kimenetek:
    visualizations/ könyvtár - PDF és PNG formátumban
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import pandas as pd
import numpy as np
from pathlib import Path

# Magyar karakterek támogatása matplotlib-ben
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['axes.unicode_minus'] = False

# Stílus beállítása - modern, professzionális megjelenés
plt.style.use('seaborn-v0_8-darkgrid')
colors = ['#2E86AB', '#A23B72', '#F18F01']  # Kék, lila, narancs (T1, T2, T3)

class TestMetricsVisualizer:
    """
    Tesztelési metrikák vizualizáló osztály.
    
    Ez az osztály felelős a három teszt (T1, T2, T3) eredményeinek
    vizualizálásáért. Különböző típusú grafikonokat generál és
    PDF/PNG formátumban menti őket.
    
    Attributes:
        output_dir (Path): Kimeneti könyvtár a grafikonoknak
        test_data (dict): T1, T2, T3 tesztek adatai és metrikái
    """
    
    def __init__(self, output_dir='visualizations'):
        """
        Vizualizátor inicializálása - kimeneti könyvtár és teszt adatok.
        
        Args:
            output_dir (str): Kimeneti könyvtár neve (alapértelmezett: visualizations)
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)  # Könyvtár létrehozása ha nem létezik
        
        # T1, T2, T3 tesztek adatai
        # Ezek az értékek a valós tesztekből származnak
        self.test_data = {
            'T1': {
                'name': 'T1: Statikus környezet',
                'duration': 60.0,  # Teszt időtartama (sec)
                'scan_rate': 0.92,  # LIDAR scan frekvencia (Hz)
                'objects_detected': 51,  # Összes detektált objektum
                'avg_objects_per_scan': 1.0,  # Átlagos objektum/scan
                'success_rate': 92.7,  # Sikeres detektálás aránya (%)
                'description': 'Egyetlen statikus objektum'
            },
            'T2': {
                'name': 'T2: Mozgó robot',
                'duration': 246.0,  # Hosszabb teszt
                'scan_rate': 0.86,  # Alacsonyabb scan rate (mozgás miatt)
                'objects_detected': 237,  # Több scan = több detektálás
                'avg_objects_per_scan': 3.5,  # Több objektum látható
                'success_rate': 95.0,  # Magas megbízhatóság
                'description': 'Mozgó robot, változó távolságok'
            },
            'T3': {
                'name': 'T3: Stressz teszt',
                'duration': 81.7,  # Közepes időtartam
                'scan_rate': 1.11,  # Legmagasabb scan rate
                'objects_detected': 1058,  # Legtöbb objektum (több objektum/scan)
                'avg_objects_per_scan': 10.26,  # Jelentősen több objektum
                'success_rate': 100.0,  # Tökéletes detektálás
                'description': 'Többszörös objektumok, dinamikus környezet'
            }
        }
    
    def plot_scan_rate_comparison(self):
        """
        LIDAR Scan Rate összehasonlító oszlopdiagram generálása.
        
        Ez a grafikon mutatja hogy a három teszt során milyen
        frekvenciával dolgozott a LIDAR sensor (mérés/sec).
        
        Magasabb scan rate = több adat, gyorsabb reaktív képesség
        """
        fig, ax = plt.subplots(figsize=(10, 6))
        
        # Teszt nevek és scan rate értékek kinyerése
        tests = list(self.test_data.keys())
        scan_rates = [self.test_data[t]['scan_rate'] for t in tests]
        
        # Oszlopdiagram létrehozása egyedi színekkel
        bars = ax.bar(tests, scan_rates, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
        
        # Értékek kiírása az oszlopok tetejére
        for bar, rate in zip(bars, scan_rates):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{rate:.2f} Hz',
                   ha='center', va='bottom', fontsize=12, fontweight='bold')
        
        # Tengelyek és cím formázása
        ax.set_ylabel('Scan Rate (Hz)', fontsize=14, fontweight='bold')
        ax.set_xlabel('Teszt Forgatókönyv', fontsize=14, fontweight='bold')
        ax.set_title('LIDAR Scan Rate Összehasonlítás', fontsize=16, fontweight='bold', pad=20)
        ax.set_ylim(0, max(scan_rates) * 1.2)  # 20% extra hely az értékeknek
        ax.grid(axis='y', alpha=0.3)
        
        # Teszt leírások hozzáadása az X tengely alá
        descriptions = [self.test_data[t]['description'] for t in tests]
        for i, (test, desc) in enumerate(zip(tests, descriptions)):
            ax.text(i, -0.15, desc, ha='center', va='top', fontsize=9, 
                   transform=ax.get_xaxis_transform(), style='italic')
        
        plt.tight_layout()
        self._save_figure(fig, 'scan_rate_comparison')
        plt.close()
        print("✅ Scan rate összehasonlítás elkészült")
    
    def plot_detection_success(self):
        """
        Detektálási sikerességi arány oszlopdiagram.
        
        Mutatja hogy a három teszt során hány százalékban sikerült
        objektumot detektálni (legalább 1 objektum/scan).
        
        100% = minden scan-ben volt detektálás (ideális)
        """
        fig, ax = plt.subplots(figsize=(10, 6))
        
        tests = list(self.test_data.keys())
        success_rates = [self.test_data[t]['success_rate'] for t in tests]
        
        bars = ax.bar(tests, success_rates, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
        
        # Értékek kiírása az oszlopokra
        for bar, rate in zip(bars, success_rates):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{rate:.1f}%',
                   ha='center', va='bottom', fontsize=12, fontweight='bold')
        
        # Tengelyek és formázás
        ax.set_ylabel('Sikeres Detektálás (%)', fontsize=14, fontweight='bold')
        ax.set_xlabel('Teszt Forgatókönyv', fontsize=14, fontweight='bold')
        ax.set_title('Objektum Detektálás Megbízhatósága', fontsize=16, fontweight='bold', pad=20)
        ax.set_ylim(0, 105)  # Kis extra hely 100% felett
        # Célvonal 100%-nál - ideális eredmény jelzése
        ax.axhline(y=100, color='green', linestyle='--', linewidth=2, alpha=0.5, label='100% Cél')
        ax.grid(axis='y', alpha=0.3)
        ax.legend(loc='lower right', fontsize=10)
        
        plt.tight_layout()
        self._save_figure(fig, 'detection_success_rate')
        plt.close()
        print("✅ Detektálási sikerességi arány elkészült")
    
    def plot_objects_per_scan(self):
        """
        Átlagos objektumszám/scan oszlopdiagram.
        
        Mutatja hogy scan-enként átlagosan hány objektumot detektált
        a rendszer. Ez a rendszer kapacitását és a teszt komplexitását
        jelzi.
        
        Magasabb érték = komplexebb környezet, több objektum kezelése
        """
        fig, ax = plt.subplots(figsize=(10, 6))
        
        tests = list(self.test_data.keys())
        avg_objects = [self.test_data[t]['avg_objects_per_scan'] for t in tests]
        
        bars = ax.bar(tests, avg_objects, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
        
        # Értékek kiírása
        for bar, obj in zip(bars, avg_objects):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{obj:.2f}',
                   ha='center', va='bottom', fontsize=12, fontweight='bold')
        
        ax.set_ylabel('Átlagos Objektumszám / Scan', fontsize=14, fontweight='bold')
        ax.set_xlabel('Teszt Forgatókönyv', fontsize=14, fontweight='bold')
        ax.set_title('Detektált Objektumok Száma Scan-enként', fontsize=16, fontweight='bold', pad=20)
        ax.set_ylim(0, max(avg_objects) * 1.2)
        ax.grid(axis='y', alpha=0.3)
        
        plt.tight_layout()
        self._save_figure(fig, 'objects_per_scan')
        plt.close()
        print("✅ Objektumszám/scan grafikon elkészült")
    
    def plot_combined_metrics(self):
        """Kombinált metrikák - 2x2 subplot"""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('LIDAR Object Detection - Teljes Teszt Összehasonlítás', 
                     fontsize=18, fontweight='bold', y=0.995)
        
        tests = list(self.test_data.keys())
        
        # 1. Scan Rate
        scan_rates = [self.test_data[t]['scan_rate'] for t in tests]
        ax1.bar(tests, scan_rates, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
        for i, rate in enumerate(scan_rates):
            ax1.text(i, rate, f'{rate:.2f} Hz', ha='center', va='bottom', fontweight='bold')
        ax1.set_ylabel('Scan Rate (Hz)', fontweight='bold')
        ax1.set_title('Scan Sebesség', fontweight='bold', fontsize=14)
        ax1.grid(axis='y', alpha=0.3)
        
        # 2. Success Rate
        success_rates = [self.test_data[t]['success_rate'] for t in tests]
        ax2.bar(tests, success_rates, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
        for i, rate in enumerate(success_rates):
            ax2.text(i, rate, f'{rate:.1f}%', ha='center', va='bottom', fontweight='bold')
        ax2.set_ylabel('Siker Arány (%)', fontweight='bold')
        ax2.set_title('Detektálási Megbízhatóság', fontweight='bold', fontsize=14)
        ax2.set_ylim(0, 105)
        ax2.axhline(y=100, color='green', linestyle='--', linewidth=2, alpha=0.5)
        ax2.grid(axis='y', alpha=0.3)
        
        # 3. Objektumok/scan
        avg_objects = [self.test_data[t]['avg_objects_per_scan'] for t in tests]
        ax3.bar(tests, avg_objects, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
        for i, obj in enumerate(avg_objects):
            ax3.text(i, obj, f'{obj:.2f}', ha='center', va='bottom', fontweight='bold')
        ax3.set_ylabel('Objektumszám', fontweight='bold')
        ax3.set_title('Átlagos Objektumok / Scan', fontweight='bold', fontsize=14)
        ax3.grid(axis='y', alpha=0.3)
        
        # 4. Teljes detektált objektumok
        total_objects = [self.test_data[t]['objects_detected'] for t in tests]
        ax4.bar(tests, total_objects, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
        for i, obj in enumerate(total_objects):
            ax4.text(i, obj, f'{obj}', ha='center', va='bottom', fontweight='bold')
        ax4.set_ylabel('Teljes Objektumszám', fontweight='bold')
        ax4.set_title('Összes Detektált Objektum', fontweight='bold', fontsize=14)
        ax4.grid(axis='y', alpha=0.3)
        ax4.set_yscale('log')
        
        plt.tight_layout()
        self._save_figure(fig, 'combined_metrics')
        plt.close()
        print("✅ Kombinált metrikák grafikon elkészült")
    
    def plot_t3_object_distribution(self, csv_path='T3_stress/t3_objects_analysis.csv'):
        """T3 objektumszám eloszlás hisztogram"""
        csv_file = Path(csv_path)
        if not csv_file.exists():
            print(f"⚠️  CSV nem található: {csv_file}")
            return
        
        # CSV beolvasása
        df = pd.read_csv(csv_file)
        object_counts = df['object_count'].values
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
        fig.suptitle('T3 Stressz Teszt - Objektum Detektálás Részletei', 
                     fontsize=16, fontweight='bold')
        
        # 1. Hisztogram
        counts, bins, patches = ax1.hist(object_counts, bins=range(7, 14), 
                                         color=colors[2], alpha=0.7, 
                                         edgecolor='black', linewidth=1.5)
        
        # Színezés értékek szerint
        for patch, count in zip(patches, counts):
            if count == max(counts):
                patch.set_facecolor('#F18F01')  # Leggyakoribb
                patch.set_alpha(0.9)
        
        ax1.set_xlabel('Objektumszám / Scan', fontsize=12, fontweight='bold')
        ax1.set_ylabel('Scan-ek Száma', fontsize=12, fontweight='bold')
        ax1.set_title('Objektumszám Eloszlás', fontweight='bold', fontsize=14)
        ax1.grid(axis='y', alpha=0.3)
        
        # Statisztikák hozzáadása
        stats_text = f"Átlag: {object_counts.mean():.2f}\n"
        stats_text += f"Medián: {np.median(object_counts):.0f}\n"
        stats_text += f"Min: {object_counts.min()}\n"
        stats_text += f"Max: {object_counts.max()}\n"
        stats_text += f"Szórás: {object_counts.std():.2f}"
        
        ax1.text(0.98, 0.97, stats_text, transform=ax1.transAxes,
                verticalalignment='top', horizontalalignment='right',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                fontsize=10, fontfamily='monospace')
        
        # 2. Idősoros ábra
        ax2.plot(df['scan_id'], df['object_count'], 
                marker='o', markersize=4, linewidth=1.5,
                color=colors[2], alpha=0.7, label='Detektált objektumok')
        ax2.axhline(y=object_counts.mean(), color='red', 
                   linestyle='--', linewidth=2, alpha=0.7, 
                   label=f'Átlag: {object_counts.mean():.2f}')
        ax2.fill_between(df['scan_id'], 
                        object_counts.mean() - object_counts.std(),
                        object_counts.mean() + object_counts.std(),
                        alpha=0.2, color=colors[2], label='±1 szórás')
        
        ax2.set_xlabel('Scan ID', fontsize=12, fontweight='bold')
        ax2.set_ylabel('Objektumszám', fontsize=12, fontweight='bold')
        ax2.set_title('Objektumszám Időbeli Változása', fontweight='bold', fontsize=14)
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='best', fontsize=10)
        ax2.set_ylim(7, 13)
        
        plt.tight_layout()
        self._save_figure(fig, 't3_object_distribution')
        plt.close()
        print("✅ T3 objektum eloszlás grafikon elkészült")
    
    def plot_performance_radar(self):
        """Radar chart - teljesítmény összehasonlítás"""
        from math import pi
        
        fig, ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(projection='polar'))
        
        # Kategóriák (normalizált értékek 0-100)
        categories = ['Scan Rate', 'Megbízhatóság', 'Obj/Scan\nKapacitás', 
                     'Teszt Időtartam', 'Összteljesítmény']
        N = len(categories)
        
        # Értékek normalizálása
        def normalize_values(test_key):
            sr = (self.test_data[test_key]['scan_rate'] / 1.11) * 100  # T3 a max
            success = self.test_data[test_key]['success_rate']
            obj_cap = (self.test_data[test_key]['avg_objects_per_scan'] / 10.26) * 100  # T3 a max
            duration = (self.test_data[test_key]['duration'] / 246) * 100  # T2 a leghosszabb
            overall = (sr + success + obj_cap) / 3  # Átlagos teljesítmény
            return [sr, success, obj_cap, duration, overall]
        
        # Szögek a radar charthoz
        angles = [n / float(N) * 2 * pi for n in range(N)]
        angles += angles[:1]
        
        ax.set_theta_offset(pi / 2)
        ax.set_theta_direction(-1)
        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(categories, fontsize=11, fontweight='bold')
        ax.set_ylim(0, 120)
        
        # Minden teszthez egy görbe
        for i, (test_key, color) in enumerate(zip(['T1', 'T2', 'T3'], colors)):
            values = normalize_values(test_key)
            values += values[:1]
            
            ax.plot(angles, values, 'o-', linewidth=2.5, 
                   color=color, label=self.test_data[test_key]['name'], alpha=0.8)
            ax.fill(angles, values, alpha=0.15, color=color)
        
        ax.set_title('Teljesítmény Radar Összehasonlítás\n(Normalizált Értékek)', 
                    fontsize=16, fontweight='bold', pad=30)
        ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1), fontsize=11)
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        self._save_figure(fig, 'performance_radar')
        plt.close()
        print("✅ Teljesítmény radar chart elkészült")
    
    def generate_summary_table(self):
        """Összefoglaló táblázat generálása (CSV és képként)"""
        # DataFrame létrehozása
        data = []
        for test_key in ['T1', 'T2', 'T3']:
            t = self.test_data[test_key]
            data.append({
                'Teszt': test_key,
                'Forgatókönyv': t['description'],
                'Időtartam (s)': f"{t['duration']:.1f}",
                'Scan Rate (Hz)': f"{t['scan_rate']:.2f}",
                'Detektált Obj.': t['objects_detected'],
                'Átlag Obj/Scan': f"{t['avg_objects_per_scan']:.2f}",
                'Sikerességi Arány (%)': f"{t['success_rate']:.1f}"
            })
        
        df = pd.DataFrame(data)
        
        # CSV mentés
        csv_path = self.output_dir / 'metrics_summary.csv'
        df.to_csv(csv_path, index=False, encoding='utf-8')
        print(f"✅ Összefoglaló táblázat CSV: {csv_path}")
        
        # Táblázat képként
        fig, ax = plt.subplots(figsize=(14, 4))
        ax.axis('tight')
        ax.axis('off')
        
        table = ax.table(cellText=df.values, colLabels=df.columns,
                        cellLoc='center', loc='center',
                        colColours=['lightblue']*len(df.columns))
        
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1, 2.5)
        
        # Header formázás
        for i in range(len(df.columns)):
            table[(0, i)].set_facecolor('#2E86AB')
            table[(0, i)].set_text_props(weight='bold', color='white')
        
        # Sorok színezése
        for i in range(1, len(df) + 1):
            for j in range(len(df.columns)):
                if i % 2 == 0:
                    table[(i, j)].set_facecolor('#f0f0f0')
        
        plt.title('Tesztelési Metrikák Összefoglaló Táblázata', 
                 fontsize=16, fontweight='bold', pad=20)
        plt.tight_layout()
        self._save_figure(fig, 'metrics_summary_table')
        plt.close()
        print("✅ Összefoglaló táblázat kép elkészült")
    
    def _save_figure(self, fig, name):
        """
        Grafikon mentése PDF és PNG formátumban.
        
        Minden grafikont két formátumban ment:
        - PDF: Vektorgrafikus, kiváló minőség nyomtatáshoz és prezentációhoz
        - PNG: Rasztergrafikus, web és dokumentáció számára
        
        Args:
            fig: Matplotlib figure objektum
            name (str): Fájlnév (kiterjesztés nélkül)
        """
        pdf_path = self.output_dir / f'{name}.pdf'
        png_path = self.output_dir / f'{name}.png'
        
        # PDF mentés - 300 DPI, szoros layout
        fig.savefig(pdf_path, dpi=300, bbox_inches='tight')
        # PNG mentés - 150 DPI (kisebb fájlméret, web-hez elegendő)
        fig.savefig(png_path, dpi=150, bbox_inches='tight')
        
        # Mentett fájlok kiírása
        print(f"   💾 {pdf_path}")
        print(f"   💾 {png_path}")
    
    def generate_all(self):
        """
        Összes grafikon és táblázat generálása.
        
        Ez a főmetódus sorban meghívja az összes vizualizációs
        metódust és legenerálja a teljes riportot.
        
        Generált kimenetek:
        1. Scan rate összehasonlítás
        2. Detektálási megbízhatóság
        3. Objektumok/scan
        4. Kombinált metrikák (2x2)
        5. T3 objektum eloszlás
        6. Teljesítmény radar chart
        7. Összefoglaló táblázat (CSV + kép)
        """
        print("\n🎨 Metrikák Vizualizáció Generálása...")
        print("=" * 50)
        
        # Minden vizualizáció generálása sorban
        self.plot_scan_rate_comparison()
        self.plot_detection_success()
        self.plot_objects_per_scan()
        self.plot_combined_metrics()
        self.plot_t3_object_distribution()
        self.plot_performance_radar()
        self.generate_summary_table()
        
        # Befejezés üzenet
        print("\n" + "=" * 50)
        print(f"✅ Összes grafikon elkészült!")
        print(f"📁 Kimenet: {self.output_dir.absolute()}")
        print("=" * 50)


if __name__ == '__main__':
    """
    Főprogram - vizualizátor futtatása.
    
    Létrehozza a TestMetricsVisualizer példányt és legenerálja
    az összes grafikont a visualizations/ könyvtárba.
    """
    # Vizualizátor létrehozása
    viz = TestMetricsVisualizer(output_dir='visualizations')
    
    # Összes grafikon generálása
    viz.generate_all()
    
    # Befejezési üzenet felhasználónak
    print("\n✅ KÉSZ! Használd a PDF fájlokat a dokumentációban és prezentációban!")
