#!/usr/bin/env python3
"""
ESDF Visualizer from XML file
Reads ESDF data saved by map_random_m node and visualizes it
"""

import numpy as np
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
import argparse
import os

class ESFDVisualizer:
    def __init__(self, xml_path):
        """Initialize the ESDF visualizer with XML file path."""
        self.xml_path = xml_path
        self.width = 0
        self.height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.frame = ""
        self.esdf_data = None
        self.occupancy_grid = None

    def load_xml(self):
        """Load ESDF data from XML file."""
        if not os.path.exists(self.xml_path):
            print(f"Error: File not found: {self.xml_path}")
            return False

        try:
            print(f"Loading ESDF from: {self.xml_path}")
            tree = ET.parse(self.xml_path)
            root = tree.getroot()

            # Parse metadata
            metadata = root.find('Metadata')
            self.width = int(metadata.find('width').text)
            self.height = int(metadata.find('height').text)
            self.resolution = float(metadata.find('resolution').text)
            self.origin_x = float(metadata.find('origin_x').text)
            self.origin_y = float(metadata.find('origin_y').text)
            self.frame = metadata.find('frame').text

            print(f"Map metadata:")
            print(f"  Size: {self.width} x {self.height}")
            print(f"  Resolution: {self.resolution} m")
            print(f"  Origin: ({self.origin_x}, {self.origin_y})")
            print(f"  Frame: {self.frame}")

            # Parse ESDF data
            esdf_data_element = root.find('ESDF_Data')
            esdf_text = esdf_data_element.text.strip()

            # Parse CSV-like data
            rows = []
            for line in esdf_text.split('\n'):
                if line.strip():
                    row = [float(x) for x in line.split(',')]
                    rows.append(row)

            self.esdf_data = np.array(rows)

            print(f"ESDF data loaded: shape={self.esdf_data.shape}")
            print(f"  Min distance: {self.esdf_data.min():.3f} m")
            print(f"  Max distance: {self.esdf_data.max():.3f} m")
            print(f"  Mean distance: {self.esdf_data.mean():.3f} m")

            # Create occupancy grid from ESDF (negative = occupied)
            self.occupancy_grid = (self.esdf_data < 0).astype(np.uint8)
            occupied_cells = np.sum(self.occupancy_grid)
            total_cells = self.occupancy_grid.size
            print(f"  Occupied cells: {occupied_cells}/{total_cells} ({100*occupied_cells/total_cells:.2f}%)")

            return True

        except Exception as e:
            print(f"Error loading XML: {e}")
            import traceback
            traceback.print_exc()
            return False

    def visualize(self, save_path=None):
        """Visualize the ESDF data."""
        if self.esdf_data is None:
            print("Error: No data loaded!")
            return

        # Calculate extent in world coordinates
        extent = [
            self.origin_x,
            self.origin_x + self.width * self.resolution,
            self.origin_y,
            self.origin_y + self.height * self.resolution
        ]

        # Create figure with single ESDF heatmap
        fig, ax = plt.subplots(1, 1, figsize=(10, 10))

        # ESDF Heatmap
        vmax = min(self.esdf_data.max(), 10)  # Cap at 10m for better visualization
        vmin = max(self.esdf_data.min(), -5)   # Cap at -5m for better visualization
        im = ax.imshow(self.esdf_data, cmap='jet_r', origin='lower', extent=extent,
                       vmin=vmin, vmax=vmax)
        # ax.set_title('ESDF (Euclidean Signed Distance Field)\n(Red=Inside obstacle, Blue=Far from obstacle)',
        #              fontsize=18)
        ax.set_xlabel('X (m)', fontsize=18)
        ax.tick_params(axis='x', labelsize=18)
        ax.set_ylabel('Y (m)', fontsize=18)
        ax.tick_params(axis='y', labelsize=18)
        ax.grid(True, alpha=0.3)
        cbar = plt.colorbar(im, ax=ax, shrink=0.8)
        cbar.set_label('Distance (m)', fontsize=18)
        cbar.ax.tick_params(labelsize=18)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Figure saved to: {save_path}")

        plt.show()

    def export_stats(self, output_path=None):
        """Export ESDF statistics to text file."""
        if self.esdf_data is None:
            print("Error: No data loaded!")
            return

        stats_text = f"""ESDF Statistics Report
{'='*50}

Map Information:
  - Size: {self.width} x {self.height} cells
  - Resolution: {self.resolution} m/cell
  - Physical size: {self.width * self.resolution} x {self.height * self.resolution} m
  - Origin: ({self.origin_x}, {self.origin_y})
  - Frame: {self.frame}

ESDF Statistics:
  - Min distance: {self.esdf_data.min():.4f} m
  - Max distance: {self.esdf_data.max():.4f} m
  - Mean distance: {self.esdf_data.mean():.4f} m
  - Std deviation: {self.esdf_data.std():.4f} m

Occupancy:
  - Total cells: {self.occupancy_grid.size}
  - Occupied cells: {np.sum(self.occupancy_grid)}
  - Free cells: {np.sum(self.occupancy_grid == 0)}
  - Occupancy rate: {100*np.sum(self.occupancy_grid)/self.occupancy_grid.size:.2f}%

Distance Distribution:
  - Cells with dist < 0m (inside obstacles): {np.sum(self.esdf_data < 0)}
  - Cells with 0m <= dist < 1m: {np.sum((self.esdf_data >= 0) & (self.esdf_data < 1))}
  - Cells with 1m <= dist < 2m: {np.sum((self.esdf_data >= 1) & (self.esdf_data < 2))}
  - Cells with 2m <= dist < 5m: {np.sum((self.esdf_data >= 2) & (self.esdf_data < 5))}
  - Cells with dist >= 5m: {np.sum(self.esdf_data >= 5)}
"""

        print(stats_text)

        if output_path:
            with open(output_path, 'w') as f:
                f.write(stats_text)
            print(f"\nStatistics saved to: {output_path}")


def main():
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_xml_path = os.path.join(script_dir, 'esdf_map.xml')

    parser = argparse.ArgumentParser(description='Visualize ESDF data from XML file')
    parser.add_argument('--xml', type=str, default=default_xml_path,
                       help=f'Path to ESDF XML file (default: {default_xml_path})')
    parser.add_argument('--save', type=str, default=None,
                       help='Path to save the visualization (e.g., output.png)')
    parser.add_argument('--stats', type=str, default=None,
                       help='Path to save statistics report (e.g., stats.txt)')

    args = parser.parse_args()

    print("=" * 70)
    print("ESDF Visualizer from XML")
    print("=" * 70)
    print()

    visualizer = ESFDVisualizer(args.xml)

    if not visualizer.load_xml():
        print("\nFailed to load XML file!")
        return

    print("\n" + "=" * 70)
    print("Generating visualization...")
    print("=" * 70)

    visualizer.visualize(save_path=args.save)

    if args.stats:
        visualizer.export_stats(output_path=args.stats)

    print("\n" + "=" * 70)
    print("Done! Close the plot window to exit.")
    print("=" * 70)


if __name__ == '__main__':
    main()
